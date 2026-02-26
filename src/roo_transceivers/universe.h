#pragma once

#include <vector>

#include "roo_collections/flat_small_hash_map.h"
#include "roo_collections/flat_small_hash_set.h"
#include "roo_logging.h"
#include "roo_transceivers.h"
#include "roo_transceivers.pb.h"
#include "roo_transceivers/id.h"
#include "roo_transceivers/notification.h"

namespace roo_transceivers {

/// An abstract collection of transceiver devices.
///
/// Each transceiver can have up to 16 sensors and up to 16 actuators, as
/// defined in its descriptor.
class Universe {
 public:
  virtual ~Universe() = default;

  /// Returns the total number of transceiver devices in this universe.
  virtual size_t deviceCount() const = 0;

  /// Iterates over all transceiver devices in this universe, calling
  /// `callback` for each device.
  ///
  /// Callback return value controls iteration:
  /// - `true`  => continue iterating,
  /// - `false` => interrupt iteration.
  ///
  /// Returns `true` if iteration completed (callback returned `true` for all
  /// devices), and `false` if interrupted by callback.
  virtual bool forEachDevice(
      std::function<bool(const DeviceLocator&)> callback) const = 0;

  /// Retrieves the descriptor for the transceiver identified by `locator`.
  ///
  /// Returns `true` on success; `false` when the device is not found.
  virtual bool getDeviceDescriptor(
      const DeviceLocator& locator,
      roo_transceivers_Descriptor& descriptor) const = 0;

  /// Returns the latest known reading of the sensor identified by `locator`.
  ///
  /// If device/sensor is not found, returned measurement is initial
  /// (`isInitial() == true`).
  virtual Measurement read(const SensorLocator& locator) const = 0;

  /// Writes to the actuator identified by `locator`.
  ///
  /// Returns `true` on success; `false` if locator is invalid or not found, or
  /// if write fails.
  virtual bool write(const ActuatorLocator& locator, float value) = 0;

  /// Requests sensor reading update from underlying devices.
  virtual void requestUpdate() = 0;

  /// Registers a listener for device-set and reading update events.
  virtual void addEventListener(EventListener* listener) {}

  /// Removes a previously registered event listener.
  virtual void removeEventListener(EventListener* listener) {}
};

}  // namespace roo_transceivers
