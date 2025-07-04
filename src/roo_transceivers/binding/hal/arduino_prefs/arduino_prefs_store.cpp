#include "roo_transceivers/binding/hal/arduino_prefs/arduino_prefs_store.h"

#include <cstdio>

#include "roo_backport.h"
#include "roo_backport/string_view.h"

namespace roo_transceivers {

namespace {

void id2key(int id, char* key) { sprintf(key, "r_%d", id); }

}  // namespace

using Rep = char[64];

SensorLocator ArduinoPreferencesBindingStore::getSensorBinding(SensorKey key) {
  roo_prefs::Transaction t(collection_, true);
  char skey[16];
  id2key(key, skey);
  Rep rep;
  roo_prefs::ReadResult result = t.store().readObject(skey, rep);
  if (result != roo_prefs::READ_OK) {
    return SensorLocator();
  }
  return SensorLocator(roo::string_view(&rep[0], 16),
                       roo::string_view(&rep[16], 24),
                       roo::string_view(&rep[40], 24));
}

void ArduinoPreferencesBindingStore::setSensorBinding(
    SensorKey key, const SensorLocator& locator) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  Rep rep;
  memset(&rep[0], 0, 64);
  strncpy(&rep[0], locator.schema().c_str(), 16);
  strncpy(&rep[16], locator.device_id().c_str(), 24);
  strncpy(&rep[40], locator.sensor_id().c_str(), 24);
  t.store().writeObject(skey, rep);
}

void ArduinoPreferencesBindingStore::clearSensorBinding(SensorKey key) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  t.store().clear(skey);
}

ActuatorLocator ArduinoPreferencesBindingStore::getActuatorBinding(
    ActuatorKey key) {
  roo_prefs::Transaction t(collection_, true);
  char skey[16];
  id2key(key, skey);
  Rep rep;
  roo_prefs::ReadResult result = t.store().readObject(skey, rep);
  if (result != roo_prefs::READ_OK) {
    return ActuatorLocator();
  }
  return ActuatorLocator(roo::string_view(&rep[0], 16),
                         roo::string_view(&rep[16], 24),
                         roo::string_view(&rep[40], 24));
}

void ArduinoPreferencesBindingStore::setActuatorBinding(
    ActuatorKey key, const ActuatorLocator& locator) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  Rep rep;
  memset(&rep[0], 0, 64);
  strncpy(&rep[0], locator.schema().c_str(), 16);
  strncpy(&rep[16], locator.device_id().c_str(), 24);
  strncpy(&rep[40], locator.actuator_id().c_str(), 24);
  t.store().writeObject(skey, rep);
}

void ArduinoPreferencesBindingStore::clearActuatorBinding(ActuatorKey key) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  t.store().clear(skey);
}

DeviceLocator ArduinoPreferencesBindingStore::getDeviceBinding(DeviceKey key) {
  roo_prefs::Transaction t(collection_, true);
  char skey[16];
  id2key(key, skey);
  Rep rep;
  roo_prefs::ReadResult result = t.store().readObject(skey, rep);
  if (result != roo_prefs::READ_OK) {
    return DeviceLocator();
  }
  return DeviceLocator(roo::string_view(&rep[0], 16),
                       roo::string_view(&rep[16], 24));
}

void ArduinoPreferencesBindingStore::setDeviceBinding(
    DeviceKey key, const DeviceLocator& locator) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  Rep rep;
  memset(&rep[0], 0, 40);
  strncpy(&rep[0], locator.schema().c_str(), 16);
  strncpy(&rep[16], locator.device_id().c_str(), 24);
  t.store().writeObject(skey, rep);
}

void ArduinoPreferencesBindingStore::clearDeviceBinding(DeviceKey key) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  t.store().clear(skey);
}

}  // namespace roo_transceivers
