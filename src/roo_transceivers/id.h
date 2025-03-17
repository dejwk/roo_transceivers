#pragma once

#include <string>

#include "roo_collections/hash.h"
#include "roo_collections/small_string.h"
#include "roo_logging.h"

namespace roo_transceivers {

using DeviceSchema = roo_collections::SmallString<16>;
using DeviceId = roo_collections::SmallString<24>;
using SensorId = roo_collections::SmallString<24>;
using ActuatorId = roo_collections::SmallString<24>;

class DeviceLocator {
 public:
  DeviceLocator();

  DeviceLocator(roo::string_view schema, roo::string_view device_id);

  const DeviceSchema& schema() const { return schema_; }
  const DeviceId& device_id() const { return device_id_; }

  bool isDefined() const { return !schema_.empty(); }

  void write_cstr(char* buf) const;

 private:
  DeviceSchema schema_;
  DeviceId device_id_;
};

inline bool operator==(const DeviceLocator& a,
                       const DeviceLocator& b) {
  return a.schema() == b.schema() && a.device_id() == b.device_id();
}

inline bool operator!=(const DeviceLocator& a,
                       const DeviceLocator& b) {
  return !(a == b);
}

class SensorLocator {
 public:
  SensorLocator();

  SensorLocator(roo::string_view schema, roo::string_view device_id,
                           roo::string_view sensor_id);

  SensorLocator(const DeviceLocator& device_loc,
                           roo::string_view sensor_id);

  const DeviceLocator& device_locator() const {
    return device_locator_;
  }

  const DeviceSchema& schema() const {
    return device_locator_.schema();
  }

  const DeviceId& device_id() const {
    return device_locator_.device_id();
  }

  const SensorId& sensor_id() const { return sensor_id_; }

  bool isDefined() const { return device_locator().isDefined(); }

  void write_cstr(char* buf) const;

 private:
  DeviceLocator device_locator_;
  SensorId sensor_id_;
};

inline bool operator==(const SensorLocator& a,
                       const SensorLocator& b) {
  return a.device_locator() == b.device_locator() &&
         a.sensor_id() == b.sensor_id();
}

class ActuatorLocator {
 public:
  ActuatorLocator();

  ActuatorLocator(const DeviceLocator& device_loc,
                             roo::string_view actuator_id);

  ActuatorLocator(roo::string_view schema,
                             roo::string_view device_id,
                             roo::string_view actuator_id);

  const DeviceLocator& device_locator() const {
    return device_locator_;
  }

  const DeviceSchema& schema() const {
    return device_locator_.schema();
  }

  const DeviceId& device_id() const {
    return device_locator_.device_id();
  }

  const ActuatorId& actuator_id() const { return actuator_id_; }

  bool isDefined() const { return device_locator().isDefined(); }

  void write_cstr(char* buf) const;

 private:
  DeviceLocator device_locator_;
  ActuatorId actuator_id_;
};

inline bool operator==(const ActuatorLocator& a,
                       const ActuatorLocator& b) {
  return a.device_locator() == b.device_locator() &&
         a.actuator_id() == b.actuator_id();
}

}  // namespace roo_transceivers

namespace std {

template <>
struct hash<roo_transceivers::DeviceSchema> {
  size_t operator()(const roo_transceivers::DeviceSchema& schema) const {
    return roo_collections::murmur3_32(schema.c_str(), strlen(schema.c_str()),
                                       0);
  }
};

template <>
struct hash<roo_transceivers::DeviceLocator> {
  size_t operator()(const roo_transceivers::DeviceLocator& loc) const {
    return roo_collections::murmur3_32(
        loc.device_id().c_str(), strlen(loc.device_id().c_str()),
        std::hash<roo_transceivers::DeviceSchema>()(loc.schema()));
  }
};

template <>
struct hash<roo_transceivers::SensorLocator> {
  size_t operator()(const roo_transceivers::SensorLocator& loc) const {
    return roo_collections::murmur3_32(
        loc.sensor_id().c_str(), strlen(loc.sensor_id().c_str()),
        std::hash<roo_transceivers::DeviceLocator>()(
            loc.device_locator()));
  }
};

template <>
struct hash<roo_transceivers::ActuatorLocator> {
  size_t operator()(const roo_transceivers::ActuatorLocator& loc) const {
    return roo_collections::murmur3_32(
        loc.actuator_id().c_str(), strlen(loc.actuator_id().c_str()),
        std::hash<roo_transceivers::DeviceLocator>()(
            loc.device_locator()));
  }
};

}  // namespace std

roo_logging::Stream& operator<<(
    roo_logging::Stream& s, const roo_transceivers::DeviceSchema& schema);

roo_logging::Stream& operator<<(
    roo_logging::Stream& s, const roo_transceivers::DeviceLocator& loc);

roo_logging::Stream& operator<<(
    roo_logging::Stream& s, const roo_transceivers::SensorLocator& loc);

roo_logging::Stream& operator<<(
    roo_logging::Stream& s, const roo_transceivers::ActuatorLocator& loc);
