#include "roo_transceivers/id.h"

#include <cstring>

namespace roo_transceivers {
namespace {

// Appends a bounded field without interpreting zero bytes as terminators.
char* AppendField(char* out, roo::string_view field) {
  if (!field.empty()) std::memcpy(out, field.data(), field.size());
  return out + field.size();
}

// Writes the complete locator and returns its length before the final NUL.
size_t WriteLocator(char* out, roo::string_view schema, roo::string_view device,
                    roo::string_view member = roo::string_view()) {
  char* pos = AppendField(out, schema);
  if (!schema.empty()) *pos++ = ':';
  pos = AppendField(pos, device);
  if (pos != out && !member.empty()) {
    *pos++ = '/';
    pos = AppendField(pos, member);
  }
  *pos = '\0';
  return pos - out;
}

}  // namespace

DeviceLocator::DeviceLocator() : schema_(), device_id_() {}

DeviceLocator::DeviceLocator(roo::string_view schema,
                             roo::string_view device_id)
    : schema_(schema), device_id_(device_id) {}

void DeviceLocator::write_cstr(char* buf) const {
  WriteLocator(buf, schema_.view(), device_id_.view());
}

std::string DeviceLocator::toString() const {
  char buf[DeviceSchema::kCapacity + 1 + DeviceId::kCapacity];
  return std::string(buf, WriteLocator(buf, schema_.view(), device_id_.view()));
}

SensorLocator::SensorLocator() : device_locator_(), sensor_id_() {}

SensorLocator::SensorLocator(roo::string_view schema,
                             roo::string_view device_id,
                             roo::string_view sensor_id)
    : device_locator_(schema, device_id), sensor_id_(sensor_id) {}

SensorLocator::SensorLocator(const DeviceLocator& device_loc,
                             roo::string_view sensor_id)
    : device_locator_(device_loc), sensor_id_(sensor_id) {}

void SensorLocator::write_cstr(char* buf) const {
  WriteLocator(buf, schema().view(), device_id().view(), sensor_id_.view());
}

std::string SensorLocator::toString() const {
  char buf[DeviceSchema::kCapacity + 1 + DeviceId::kCapacity + 1 +
           SensorId::kCapacity];
  return std::string(buf, WriteLocator(buf, schema().view(), device_id().view(),
                                       sensor_id_.view()));
}

ActuatorLocator::ActuatorLocator() : device_locator_(), actuator_id_() {}

ActuatorLocator::ActuatorLocator(const DeviceLocator& device_loc,
                                 roo::string_view actuator_id)
    : device_locator_(device_loc), actuator_id_(actuator_id) {}

ActuatorLocator::ActuatorLocator(roo::string_view schema,
                                 roo::string_view device_id,
                                 roo::string_view actuator_id)
    : device_locator_(schema, device_id), actuator_id_(actuator_id) {}

void ActuatorLocator::write_cstr(char* buf) const {
  WriteLocator(buf, schema().view(), device_id().view(), actuator_id_.view());
}

std::string ActuatorLocator::toString() const {
  char buf[DeviceSchema::kCapacity + 1 + DeviceId::kCapacity + 1 +
           ActuatorId::kCapacity];
  return std::string(buf, WriteLocator(buf, schema().view(), device_id().view(),
                                       actuator_id_.view()));
}

roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const DeviceSchema& schema) {
  s << schema.view();
  return s;
}

roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const DeviceLocator& loc) {
  s << loc.schema() << ":" << loc.device_id().view();
  return s;
}

roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const SensorLocator& loc) {
  s << loc.device_locator();
  if (!loc.sensor_id().empty()) {
    s << "/" << loc.sensor_id().view();
  }
  return s;
}

roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const ActuatorLocator& loc) {
  s << loc.device_locator();
  if (!loc.actuator_id().empty()) {
    s << "/" << loc.actuator_id().view();
  }
  return s;
}

}  // namespace roo_transceivers