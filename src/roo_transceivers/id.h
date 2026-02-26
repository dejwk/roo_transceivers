#pragma once

#include <string>

#include "roo_collections/hash.h"
#include "roo_collections/small_string.h"
#include "roo_logging.h"

namespace roo_transceivers {

/// Device schema identifier (short string).
using DeviceSchema = roo_collections::SmallString<16>;
/// Device identifier (short string).
using DeviceId = roo_collections::SmallString<24>;
/// Sensor identifier (short string).
using SensorId = roo_collections::SmallString<24>;
/// Actuator identifier (short string).
using ActuatorId = roo_collections::SmallString<24>;

/// Identifies a transceiver device by schema and device id.
class DeviceLocator {
 public:
  DeviceLocator();

  DeviceLocator(roo::string_view schema, roo::string_view device_id);

  /// Returns the device schema.
  const DeviceSchema& schema() const { return schema_; }
  /// Returns the device id.
  const DeviceId& device_id() const { return device_id_; }

  /// Returns true if schema or device id is set.
  bool isDefined() const { return !schema_.empty() || !device_id().empty(); }

  /// Writes a null-terminated string representation into `buf`.
  void write_cstr(char* buf) const;

  /// Returns a string representation.
  std::string toString() const;

 private:
  DeviceSchema schema_;
  DeviceId device_id_;
};

inline bool operator==(const DeviceLocator& a, const DeviceLocator& b) {
  return a.schema() == b.schema() && a.device_id() == b.device_id();
}

inline bool operator!=(const DeviceLocator& a, const DeviceLocator& b) {
  return !(a == b);
}

/// Identifies sensor within a transceiver device.
///
/// Consists of device locator and sensor id.
class SensorLocator {
 public:
  SensorLocator();

  SensorLocator(roo::string_view schema, roo::string_view device_id,
                roo::string_view sensor_id);

  SensorLocator(const DeviceLocator& device_loc, roo::string_view sensor_id);

  /// Returns the device locator.
  const DeviceLocator& device_locator() const { return device_locator_; }

  /// Returns the device schema.
  const DeviceSchema& schema() const { return device_locator_.schema(); }

  /// Returns the device id.
  const DeviceId& device_id() const { return device_locator_.device_id(); }

  /// Returns the sensor id.
  const SensorId& sensor_id() const { return sensor_id_; }

  /// Returns true if the locator is defined.
  bool isDefined() const { return device_locator().isDefined(); }

  /// Writes a null-terminated string representation into `buf`.
  void write_cstr(char* buf) const;

  /// Returns a string representation.
  std::string toString() const;

 private:
  DeviceLocator device_locator_;
  SensorId sensor_id_;
};

inline bool operator==(const SensorLocator& a, const SensorLocator& b) {
  return a.device_locator() == b.device_locator() &&
         a.sensor_id() == b.sensor_id();
}

inline bool operator!=(const SensorLocator& a, const SensorLocator& b) {
  return !(a == b);
}

/// Identifies actuator within a transceiver device.
///
/// Consists of device locator and actuator id.
class ActuatorLocator {
 public:
  ActuatorLocator();

  ActuatorLocator(const DeviceLocator& device_loc,
                  roo::string_view actuator_id);

  ActuatorLocator(roo::string_view schema, roo::string_view device_id,
                  roo::string_view actuator_id);

  /// Returns the device locator.
  const DeviceLocator& device_locator() const { return device_locator_; }

  /// Returns the device schema.
  const DeviceSchema& schema() const { return device_locator_.schema(); }

  /// Returns the device id.
  const DeviceId& device_id() const { return device_locator_.device_id(); }

  /// Returns the actuator id.
  const ActuatorId& actuator_id() const { return actuator_id_; }

  /// Returns true if the locator is defined.
  bool isDefined() const { return device_locator().isDefined(); }

  /// Writes a null-terminated string representation into `buf`.
  void write_cstr(char* buf) const;

  /// Returns a string representation.
  std::string toString() const;

 private:
  DeviceLocator device_locator_;
  ActuatorId actuator_id_;
};

inline bool operator==(const ActuatorLocator& a, const ActuatorLocator& b) {
  return a.device_locator() == b.device_locator() &&
         a.actuator_id() == b.actuator_id();
}

inline bool operator!=(const ActuatorLocator& a, const ActuatorLocator& b) {
  return !(a == b);
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
        std::hash<roo_transceivers::DeviceLocator>()(loc.device_locator()));
  }
};

template <>
struct hash<roo_transceivers::ActuatorLocator> {
  size_t operator()(const roo_transceivers::ActuatorLocator& loc) const {
    return roo_collections::murmur3_32(
        loc.actuator_id().c_str(), strlen(loc.actuator_id().c_str()),
        std::hash<roo_transceivers::DeviceLocator>()(loc.device_locator()));
  }
};

}  // namespace std

roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const roo_transceivers::DeviceSchema& schema);

/// Streams a device locator in a human-readable form.
roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const roo_transceivers::DeviceLocator& loc);

/// Streams a sensor locator in a human-readable form.
roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const roo_transceivers::SensorLocator& loc);

/// Streams an actuator locator in a human-readable form.
roo_logging::Stream& operator<<(roo_logging::Stream& s,
                                const roo_transceivers::ActuatorLocator& loc);
