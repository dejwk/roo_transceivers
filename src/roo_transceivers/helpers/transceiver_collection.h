#pragma once

#include "roo_transceivers/universe.h"

namespace roo_transceivers {

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

}