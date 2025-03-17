#pragma once

#include "roo_transceivers/id.h"

namespace roo_transceivers {

// Stores (e.g. in Preferences) the mapping from binding keys to device IDs.
class BindingStore {
 public:
  using SensorKey = uint32_t;

  virtual SensorLocator getSensorBinding(SensorKey key) = 0;
  virtual void setSensorBinding(SensorKey key,
                                const SensorLocator& locator) = 0;
  virtual void clearSensorBinding(SensorKey key) = 0;
};

}  // namespace roo_transceivers
