#pragma once

// #include <vector>

#include "roo_collections/flat_small_hash_map.h"
#include "roo_logging.h"
#include "roo_transceivers/id.h"
#include "roo_transceivers/notification.h"

namespace roo_transceivers {

class Universe {
 public:
  virtual ~Universe() = default;

  virtual int deviceCount() const = 0;

  virtual void forEachDevice(
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
  bool getDeviceDescriptor(const DeviceLocator& locator,
                           roo_transceivers_Descriptor& descriptor) const override {
    descriptor.sensors_count = 1;
    descriptor.sensors[0].id[0] = 0;
    descriptor.sensors[0].quantity = getSensorQuantity(locator);
    descriptor.actuators_count = 0;
    return true;
  }

  Measurement read(const SensorLocator& locator) const override {
    CHECK(locator.sensor_id().empty());
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

}  // namespace roo_transceivers