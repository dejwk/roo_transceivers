#include "roo_transceivers/binding/hal/arduino_prefs/arduino_prefs_store.h"

#include <cstdio>
#include <cstring>

#include "roo_backport.h"
#include "roo_backport/string_view.h"

namespace roo_transceivers {

namespace {

void id2key(int id, char* key) { sprintf(key, "r_%d", id); }

constexpr size_t kDeviceOffset = DeviceSchema::kCapacity;
constexpr size_t kMemberOffset = kDeviceOffset + DeviceId::kCapacity;
using Rep = char[kMemberOffset + SensorId::kCapacity];
static_assert(SensorId::kCapacity == ActuatorId::kCapacity,
              "Binding member fields share the same storage width");

// Legacy fields use zero padding, including the final byte. A nonzero final
// byte stores the length when the value contains an embedded zero.
template <size_t Width>
roo::string_view ReadField(const char* field) {
  const size_t binary_length = static_cast<unsigned char>(field[Width - 1]);
  if (binary_length != 0 && binary_length < Width) {
    return roo::string_view(field, binary_length);
  }
  const char* end = static_cast<const char*>(std::memchr(field, 0, Width - 1));
  return roo::string_view(field, end == nullptr ? Width - 1 : end - field);
}

// Writes into a zero-initialized field, preserving the original text format.
template <size_t Width>
void WriteField(char* field, roo::string_view value) {
  std::memcpy(field, value.data(), value.size());
  if (std::memchr(value.data(), 0, value.size()) != nullptr) {
    field[Width - 1] = static_cast<char>(value.size());
  }
}

}  // namespace

SensorLocator ArduinoPreferencesBindingStore::getSensorBinding(SensorKey key) {
  roo_prefs::Transaction t(collection_,
                           roo_prefs::Transaction::Mode::kReadOnly);
  char skey[16];
  id2key(key, skey);
  Rep rep;
  roo_prefs::ReadResult result = t.store().readObject(skey, rep);
  if (result != roo_prefs::READ_OK) {
    return SensorLocator();
  }
  return SensorLocator(ReadField<DeviceSchema::kCapacity>(&rep[0]),
                       ReadField<DeviceId::kCapacity>(&rep[kDeviceOffset]),
                       ReadField<SensorId::kCapacity>(&rep[kMemberOffset]));
}

void ArduinoPreferencesBindingStore::setSensorBinding(
    SensorKey key, const SensorLocator& locator) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  Rep rep = {};
  WriteField<DeviceSchema::kCapacity>(&rep[0], locator.schema().view());
  WriteField<DeviceId::kCapacity>(&rep[kDeviceOffset],
                                  locator.device_id().view());
  WriteField<SensorId::kCapacity>(&rep[kMemberOffset],
                                  locator.sensor_id().view());
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
  roo_prefs::Transaction t(collection_,
                           roo_prefs::Transaction::Mode::kReadOnly);
  char skey[16];
  id2key(key, skey);
  Rep rep;
  roo_prefs::ReadResult result = t.store().readObject(skey, rep);
  if (result != roo_prefs::READ_OK) {
    return ActuatorLocator();
  }
  return ActuatorLocator(ReadField<DeviceSchema::kCapacity>(&rep[0]),
                         ReadField<DeviceId::kCapacity>(&rep[kDeviceOffset]),
                         ReadField<ActuatorId::kCapacity>(&rep[kMemberOffset]));
}

void ArduinoPreferencesBindingStore::setActuatorBinding(
    ActuatorKey key, const ActuatorLocator& locator) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  Rep rep = {};
  WriteField<DeviceSchema::kCapacity>(&rep[0], locator.schema().view());
  WriteField<DeviceId::kCapacity>(&rep[kDeviceOffset],
                                  locator.device_id().view());
  WriteField<ActuatorId::kCapacity>(&rep[kMemberOffset],
                                    locator.actuator_id().view());
  t.store().writeObject(skey, rep);
}

void ArduinoPreferencesBindingStore::clearActuatorBinding(ActuatorKey key) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  t.store().clear(skey);
}

DeviceLocator ArduinoPreferencesBindingStore::getDeviceBinding(DeviceKey key) {
  roo_prefs::Transaction t(collection_,
                           roo_prefs::Transaction::Mode::kReadOnly);
  char skey[16];
  id2key(key, skey);
  Rep rep;
  roo_prefs::ReadResult result = t.store().readObject(skey, rep);
  if (result != roo_prefs::READ_OK) {
    return DeviceLocator();
  }
  return DeviceLocator(ReadField<DeviceSchema::kCapacity>(&rep[0]),
                       ReadField<DeviceId::kCapacity>(&rep[kDeviceOffset]));
}

void ArduinoPreferencesBindingStore::setDeviceBinding(
    DeviceKey key, const DeviceLocator& locator) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  Rep rep = {};
  WriteField<DeviceSchema::kCapacity>(&rep[0], locator.schema().view());
  WriteField<DeviceId::kCapacity>(&rep[kDeviceOffset],
                                  locator.device_id().view());
  t.store().writeObject(skey, rep);
}

void ArduinoPreferencesBindingStore::clearDeviceBinding(DeviceKey key) {
  roo_prefs::Transaction t(collection_);
  char skey[16];
  id2key(key, skey);
  t.store().clear(skey);
}

}  // namespace roo_transceivers
