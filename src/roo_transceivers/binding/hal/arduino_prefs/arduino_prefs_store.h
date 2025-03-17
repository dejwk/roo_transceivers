#pragma once

#include "roo_prefs.h"
#include "roo_transceivers/binding/hal/store.h"

namespace roo_transceivers {

class ArduinoPreferencesBindingStore : public BindingStore {
 public:
  ArduinoPreferencesBindingStore() : collection_("roo/ct/bindings") {}

  SensorLocator getSensorBinding(SensorKey key) override;
  void setSensorBinding(SensorKey key, const SensorLocator& locator) override;
  void clearSensorBinding(SensorKey key) override;

 private:
  roo_prefs::Collection collection_;
};

}  // namespace roo_transceivers
