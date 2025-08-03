#pragma once

#include <vector>

#include "roo_collections/flat_small_hash_map.h"
#include "roo_collections/flat_small_hash_set.h"
#include "roo_logging.h"
#include "roo_transceivers/id.h"
#include "roo_transceivers/notification.h"

namespace roo_transceivers {

// An abstract collection of transceiver devices.
//
// Each transceiver can have up to 16 sensors and up to 16 actuators, as defined
// in its descriptor.
class Universe {
 public:
  virtual ~Universe() = default;

  // Returns the total number of transceiver devices in this universe.
  virtual size_t deviceCount() const = 0;

  // Iterates over all transceiver devices in this universe, calling the
  // user-specified cllback for each of them. The iteration gets interrupted
  // when the callback returns false.
  //
  // Returns true if the iteration completed (when the callback returned true
  // for all devices), and false in case it was interrupted due to the callback
  // returning false.
  virtual bool forEachDevice(
      std::function<bool(const DeviceLocator&)> callback) const = 0;

  // Retrieves the descriptor for the transceiver device identified by the
  // specified locator. Returns true on success, false when the universe does
  // not contain a transceiver device identified with the specified locator.
  virtual bool getDeviceDescriptor(
      const DeviceLocator& locator,
      roo_transceivers_Descriptor& descriptor) const = 0;

  // Returns the latest known reading of the sensor identified by the specified
  // locator. In case the device is not found or the sensor does not exist, the
  // returned measurement will return true from 'isInitial()'.
  virtual Measurement read(const SensorLocator& locator) const = 0;

  // Writes to the actuator identified by the specified locator. Returns true on
  // success; false in case the locator does not point to a valid actuator of a
  // transceiver device within this universe, or if the operation cannot be
  // completed for some other reason.
  virtual bool write(const ActuatorLocator& locator, float value) = 0;

  // Requests that the universe updates sensor readings. For example, if the
  // universe contains thermometers, temperature conversion will be requested.
  virtual void requestUpdate() = 0;

  // Registers a listener to be notified when the set of transceivers change or
  // when new readings are available.
  virtual void addEventListener(EventListener* listener) {}

  // Removes a previously registered event listener.
  virtual void removeEventListener(EventListener* listener) {}
};

// SimpleSensorUniverse is a convenience universe that contains identical
// and basic transceivers, which all contain just one sensor and no
// transceivers. A common example is a collection of thermometers.
//
// This class implements the entire contract of the Universe, and requires
// the implementor to provide two new methods instead: readSensor to retrieve
// the sensor reading, and getSensorQuantity to return the sensor quantity (e.g.
// temperature, time, pressure, etc.).
class SimpleSensorUniverse : public Universe {
 public:
  bool getDeviceDescriptor(
      const DeviceLocator& locator,
      roo_transceivers_Descriptor& descriptor) const override {
    roo_transceivers_Quantity quantity = getSensorQuantity(locator);
    if (quantity == roo_transceivers_Quantity_kUnspecifiedQuantity) {
      return false;
    }
    descriptor.sensors_count = 1;
    descriptor.sensors[0].id[0] = 0;
    descriptor.sensors[0].quantity = quantity;
    descriptor.actuators_count = 0;
    return true;
  }

  Measurement read(const SensorLocator& locator) const override {
    if (!locator.sensor_id().empty()) {
      // It must not be a device that this universe manages.
      return Measurement();
    }
    return readSensor(locator.device_locator());
  }

  bool write(const ActuatorLocator& locator, float value) override {
    return false;
  }

 protected:
  virtual Measurement readSensor(const DeviceLocator& locator) const = 0;

  virtual roo_transceivers_Quantity getSensorQuantity(
      DeviceLocator device_locator) const = 0;
};

// Multiverse is a transceiver universe that combines multiple other universes
// into one.
class Multiverse : public Universe, public EventListener {
 public:
  Multiverse(std::vector<Universe*> universes)
      : universes_(std::move(universes)) {
    for (auto universe : universes_) {
      universe->addEventListener(this);
    }
  }

  size_t deviceCount() const override {
    size_t count = 0;
    for (const auto& universe : universes_) {
      count += universe->deviceCount();
    }
    return count;
  }

  bool forEachDevice(
      std::function<bool(const DeviceLocator&)> callback) const override {
    for (const auto& universe : universes_) {
      if (!universe->forEachDevice(callback)) return false;
    }
    return true;
  }

  bool getDeviceDescriptor(
      const DeviceLocator& locator,
      roo_transceivers_Descriptor& descriptor) const override {
    for (const auto& universe : universes_) {
      if (universe->getDeviceDescriptor(locator, descriptor)) return true;
    }
    return false;
  }

  Measurement read(const SensorLocator& locator) const override {
    for (const auto& universe : universes_) {
      Measurement m = universe->read(locator);
      if (m.quantity() != roo_transceivers_Quantity_kUnspecifiedQuantity) {
        return m;
      }
    }
    return Measurement();
  }

  bool write(const ActuatorLocator& locator, float value) override {
    for (const auto& universe : universes_) {
      if (universe->write(locator, value)) return true;
    }
    return false;
  }

  void requestUpdate() override {
    for (const auto& universe : universes_) {
      universe->requestUpdate();
    }
  }

  virtual void addEventListener(EventListener* listener) {
    listeners_.insert(listener);
  }

  virtual void removeEventListener(EventListener* listener) {
    listeners_.erase(listener);
  }

  void devicesChanged() override {
    for (auto listener : listeners_) {
      listener->devicesChanged();
    }
  }

  void newReadingsAvailable() override {
    for (auto listener : listeners_) {
      listener->newReadingsAvailable();
    }
  }

 private:
  std::vector<Universe*> universes_;
  roo_collections::FlatSmallHashSet<EventListener*> listeners_;
};

// For use with TransceiverCollection, below.
class Transceiver {
 public:
  Transceiver() = default;
  virtual ~Transceiver() = default;

  virtual void getDescriptor(roo_transceivers_Descriptor& descriptor) const = 0;

  virtual Measurement read(const SensorId& sensor) const = 0;
  virtual bool write(const ActuatorId& actuator, float value) = 0;

  virtual void requestUpdate() {}
  virtual void addEventListener(EventListener* listener) = 0;
  virtual void removeEventListener(EventListener* listener) = 0;
};

class TransceiverCollection : public Universe, public EventListener {
 public:
  struct Entry {
    DeviceLocator locator;
    Transceiver* instance;
  };

  TransceiverCollection() = default;

  TransceiverCollection(std::vector<Entry> transceivers) {
    for (auto& transceiver : transceivers) {
      add(transceiver.locator, transceiver.instance);
    }
  }

  void add(const DeviceLocator& locator, Transceiver* device) {
    CHECK(transceivers_.insert(std::make_pair(locator, device)).second)
        << "Duplicate device locator: " << locator;
    device->addEventListener(this);
  }

  size_t deviceCount() const override { return transceivers_.size(); }

  bool forEachDevice(
      std::function<bool(const DeviceLocator&)> callback) const override {
    for (const auto& transceiver : transceivers_) {
      if (!callback(transceiver.first)) return false;
    }
    return true;
  }

  bool getDeviceDescriptor(
      const DeviceLocator& locator,
      roo_transceivers_Descriptor& descriptor) const override {
    const auto& itr = transceivers_.find(locator);
    if (itr == transceivers_.end()) return false;
    itr->second->getDescriptor(descriptor);
    return true;
  }

  Measurement read(const SensorLocator& locator) const override {
    const auto& itr = transceivers_.find(locator.device_locator());
    return (itr != transceivers_.end()) ? itr->second->read(locator.sensor_id())
                                        : Measurement();
  }

  bool write(const ActuatorLocator& locator, float value) override {
    auto itr = transceivers_.find(locator.device_locator());
    return (itr != transceivers_.end())
               ? itr->second->write(locator.actuator_id(), value)
               : false;
  }

  void requestUpdate() override {
    for (const auto& transceiver : transceivers_) {
      transceiver.second->requestUpdate();
    }
  }

  virtual void addEventListener(EventListener* listener) {
    listeners_.insert(listener);
  }

  virtual void removeEventListener(EventListener* listener) {
    listeners_.erase(listener);
  }

  void devicesChanged() override {
    for (auto listener : listeners_) {
      listener->devicesChanged();
    }
  }

  void newReadingsAvailable() override {
    for (auto listener : listeners_) {
      listener->newReadingsAvailable();
    }
  }

 private:
  roo_collections::FlatSmallHashMap<DeviceLocator, Transceiver*> transceivers_;
  roo_collections::FlatSmallHashSet<EventListener*> listeners_;
};

class TransceiverType {
 public:
  TransceiverType(roo_transceivers_Descriptor descriptor)
      : descriptor_(std::move(descriptor)) {
    for (size_t i = 0; i < descriptor.sensors_count; i++) {
      sensors_[descriptor.sensors[i].id] = i;
    }
    for (size_t i = 0; i < descriptor.actuators_count; i++) {
      actuators_[descriptor.actuators[i].id] = i;
    }
  }

  const roo_transceivers_Descriptor& getDescriptor() const {
    return descriptor_;
  }

  // Returns -1 if not found.
  int resolveSensorIndex(const SensorId& id) const {
    auto itr = sensors_.find(id);
    return (itr == sensors_.end()) ? -1 : itr->second;
  }

  // Returns -1 if not found.
  int resolveActuatorIndex(const ActuatorId& id) const {
    auto itr = actuators_.find(id);
    return (itr == actuators_.end()) ? -1 : itr->second;
  }

 private:
  roo_transceivers_Descriptor descriptor_;
  roo_collections::FlatSmallHashMap<SensorId, int> sensors_;
  roo_collections::FlatSmallHashMap<ActuatorId, int> actuators_;
};

// A transceiver with a statically defined descriptor. The class provides basic
// implementations for all methods, ensuring input validation, and support for
// even listener management.
class SimpleTransceiver : public Transceiver {
 public:
  // The provided type must outlive this transceiver.
  SimpleTransceiver(const TransceiverType* type) : type_(type) {}

  void getDescriptor(roo_transceivers_Descriptor& descriptor) const override {
    descriptor = type_->getDescriptor();
  }

  Measurement read(const SensorId& sensor) const override {
    int idx = type_->resolveSensorIndex(sensor);
    if (idx < 0) return Measurement();
    return readFromSensor(idx);
  }

  bool write(const ActuatorId& actuator, float value) override {
    int idx = type_->resolveActuatorIndex(actuator);
    if (idx < 0) return false;
    return writeToActuator(idx, value);
  }

  void addEventListener(EventListener* listener) override {
    event_listeners_.insert(listener);
  }

  void removeEventListener(EventListener* listener) override {
    event_listeners_.erase(listener);
  }

 protected:
  // The subclass can assume that the idx has been validated to be in the range
  // implied by the descriptor.
  virtual Measurement readFromSensor(int idx) const = 0;

  // The subclass can assume that the idx has been validated to be in the range
  // implied by the descriptor.
  virtual bool writeToActuator(int idx, float value) = 0;

  void notifyNewReadingsAvailable() const {
    for (auto* listener : event_listeners_) {
      listener->newReadingsAvailable();
    }
  }

  const TransceiverType* type_;
  roo_collections::FlatSmallHashSet<EventListener*> event_listeners_;
};

// A transceiver with just one statically defined sensor. The class provides
// basic implementations for all methods, ensuring input validation, and support
// for even listener management.
class SimpleSensor : public Transceiver {
 public:
  // The provided type must outlive this transceiver.
  SimpleSensor(roo_transceivers_Quantity quantity, SensorId id = "")
      : quantity_(quantity), id_(id) {}

  void getDescriptor(roo_transceivers_Descriptor& descriptor) const override {
    descriptor.sensors_count = 1;
    strncpy(descriptor.sensors[0].id, id_.c_str(), SensorId::kCapacity);
    descriptor.sensors[0].quantity = quantity_;
    descriptor.actuators_count = 0;
  }

  Measurement read(const SensorId& sensor) const override {
    if (sensor != id_) return Measurement();
    return Measurement(quantity_, roo_time::Uptime::Now(), readFromSensor());
  }

  bool write(const ActuatorId& actuator, float value) override { return false; }

  void addEventListener(EventListener* listener) override {
    event_listeners_.insert(listener);
  }

  void removeEventListener(EventListener* listener) override {
    event_listeners_.erase(listener);
  }

 protected:
  virtual float readFromSensor() const = 0;

  void notifyNewReadingsAvailable() const {
    for (auto* listener : event_listeners_) {
      listener->newReadingsAvailable();
    }
  }

  roo_transceivers_Quantity quantity_;
  SensorId id_;
  roo_collections::FlatSmallHashSet<EventListener*> event_listeners_;
};

}  // namespace roo_transceivers

bool operator==(const roo_transceivers_Descriptor& a,
                const roo_transceivers_Descriptor& b);
