#include "roo_transceivers/id.h"

namespace roo_transceivers {

DeviceLocator::DeviceLocator() : schema_(), device_id_() {}

DeviceLocator::DeviceLocator(roo::string_view schema,
                             roo::string_view device_id)
    : schema_(schema), device_id_(device_id) {}

void DeviceLocator::write_cstr(char* buf) const {
  strncpy(buf, schema_.c_str(), DeviceSchema::kCapacity);
  size_t len = strlen(buf);
  if (len > 0) {
    buf[len++] = ':';
    strncpy(buf + len, device_id_.c_str(), 24);
  }
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
  device_locator_.write_cstr(buf);
  size_t len = strlen(buf);
  if (len > 0 && !sensor_id_.empty()) {
    buf[len++] = '/';
    strncpy(buf + len, sensor_id_.c_str(), 24);
  }
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
  device_locator_.write_cstr(buf);
  size_t len = strlen(buf);
  if (len > 0 && !actuator_id_.empty()) {
    buf[len++] = '/';
    strncpy(buf + len, actuator_id_.c_str(), 24);
  }
}

}  // namespace roo_transceivers

roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const roo_transceivers::DeviceSchema& schema) {
  s << schema.c_str();
  return s;
}

roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const roo_transceivers::DeviceLocator& loc) {
  s << loc.schema() << ":" << loc.device_id().c_str();
  return s;
}

roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const roo_transceivers::SensorLocator& loc) {
  s << loc.device_locator();
  if (!loc.sensor_id().empty()) {
    s << "/" << loc.sensor_id().c_str();
  }
  return s;
}

roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const roo_transceivers::ActuatorLocator& loc) {
  s << loc.device_locator();
  if (!loc.actuator_id().empty()) {
    s << "/" << loc.actuator_id().c_str();
  }
  return s;
}