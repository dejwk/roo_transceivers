#pragma once

#include "roo_transceivers/universe.h"

namespace roo_transceivers {

/// Convenience `Universe` for identical, basic sensor-only transceivers.
///
/// Each device exposes exactly one sensor and no actuators.
///
/// Can be used e.g. for a collection of identical thermometers.
///
/// This class implements the full `Universe` contract. Implementations only
/// need to provide:
/// - `readSensor()` to return reading for a device,
/// - `getSensorQuantity()` to define sensor quantity type.
class SimpleSensorUniverse : public Universe {
 public:
  /// Synthesizes a single-sensor descriptor for `locator`.
  ///
  /// Returns `false` when `locator` is not recognized by this universe.
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

  /// Reads the single sensor of a device.
  ///
  /// Only empty sensor id (`""`) is valid. Any non-empty sensor id returns an
  /// initial/unspecified measurement.
  Measurement read(const SensorLocator& locator) const override {
    if (!locator.sensor_id().empty()) {
      return Measurement();
    }
    return readSensor(locator.device_locator());
  }

  /// This universe is sensor-only; writes are always rejected.
  bool write(const ActuatorLocator& locator, float value) override {
    return false;
  }

 protected:
  virtual Measurement readSensor(const DeviceLocator& locator) const = 0;

  virtual roo_transceivers_Quantity getSensorQuantity(
      DeviceLocator device_locator) const = 0;
};

}  // namespace roo_transceivers