#pragma once

#include <vector>

#include "roo_collections/flat_small_hash_map.h"
#include "roo_collections/flat_small_hash_set.h"
#include "roo_logging.h"
#include "roo_transceivers/id.h"
#include "roo_transceivers/notification.h"

namespace roo_transceivers {

class Universe {
 public:
  virtual ~Universe() = default;

  virtual size_t deviceCount() const = 0;

  // Returns true if the iteration completed (when the callback returned true
  // for all devices), and false in case it was interrupted due to the callback
  // returning false.
  virtual bool forEachDevice(
      std::function<bool(const DeviceLocator&)> callback) const = 0;

  virtual bool getDeviceDescriptor(
      const DeviceLocator& locator,
      roo_transceivers_Descriptor& descriptor) const = 0;

  virtual Measurement read(const SensorLocator& locator) const = 0;

  virtual bool write(const ActuatorLocator& locator, float value) const = 0;

  virtual void requestUpdate() = 0;

  virtual void addEventListener(EventListener* listener) {}

  virtual void removeEventListener(EventListener* listener) {}
};

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

  bool write(const ActuatorLocator& locator, float value) const override {
    LOG(FATAL) << "This device has no actuators.";
    return false;
  }

 protected:
  virtual Measurement readSensor(const DeviceLocator& locator) const = 0;

  virtual roo_transceivers_Quantity getSensorQuantity(
      DeviceLocator device_locator) const = 0;
};

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

  bool write(const ActuatorLocator& locator, float value) const override {
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

}  // namespace roo_transceivers