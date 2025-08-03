#pragma once

#include "roo_transceivers/universe.h"

namespace roo_transceivers {

// SimpleSensorUniverse is a convenience universe that contains identical
// and basic transceivers, which all contain just one sensor and no
// actuators. A common example is a collection of thermometers.
//
// This class implements the entire contract of the Universe, and requires
// the implementor to provide two new methods instead: readSensor to retrieve
// the sensor reading, and getSensorQuantity to return the sensor quantity (e.g.
// temperature, time, pressure, etc.)
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

}