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

}  // namespace roo_transceivers
