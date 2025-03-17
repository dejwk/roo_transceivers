#pragma once

#include "roo_transceivers/id.h"

namespace roo_transceivers {

// Stores (e.g. in Preferences) the mapping from binding keys to device IDs.
class BindingStore {
 public:
  using SensorKey = uint32_t;
  using ActuatorKey = uint32_t;
  using DeviceKey = uint32_t;

  virtual SensorLocator getSensorBinding(SensorKey key) = 0;
  virtual void setSensorBinding(SensorKey key,
                                const SensorLocator& locator) = 0;
  virtual void clearSensorBinding(SensorKey key) = 0;

  virtual ActuatorLocator getActuatorBinding(ActuatorKey key) = 0;
  virtual void setActuatorBinding(ActuatorKey key,
                                  const ActuatorLocator& locator) = 0;
  virtual void clearActuatorBinding(ActuatorKey key) = 0;

  virtual DeviceLocator getDeviceBinding(DeviceKey key) = 0;
  virtual void setDeviceBinding(DeviceKey key,
                                const DeviceLocator& locator) = 0;
  virtual void clearDeviceBinding(DeviceKey key) = 0;
};

}  // namespace roo_transceivers
