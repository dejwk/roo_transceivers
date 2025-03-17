#include "roo_transceivers/binding/hal/defaults.h"

namespace roo_transceivers {

BindingStore& DefaultBindingStore() {
  static ArduinoPreferencesBindingStore store;
  return store;
}

}