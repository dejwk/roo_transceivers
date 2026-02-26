#pragma once

#include <cstdint>

#include "roo_logging.h"
#include "roo_transceivers/binding/hal/store.h"
#include "roo_transceivers/measurement.h"
#include "roo_transceivers/universe.h"

namespace roo_transceivers {

/// Persistent binding of a sensor key to a locator.
class SensorBinding {
 public:
  SensorBinding(BindingStore& store, BindingStore::SensorKey key)
      : loc_(), store_(store), key_(key), synced_(false) {}

  /// Returns the bound locator (empty if unbound).
  SensorLocator get() const {
    sync();
    return loc_;
  }

  /// Returns true when a locator is bound.
  bool isBound() const {
    sync();
    return loc_.isDefined();
  }

  /// Binds to the specified locator (or unbinds when undefined).
  void bind(const SensorLocator& loc) {
    if (loc_ == loc) return;
    loc_ = loc;
    if (!loc_.isDefined()) {
      store_.clearSensorBinding(key_);
    } else {
      store_.setSensorBinding(key_, loc_);
    }
    synced_ = true;
  }

  /// Clears the binding.
  void unbind() {
    loc_ = SensorLocator();
    store_.clearSensorBinding(key_);
    synced_ = true;
  }

 private:
  friend roo_logging::Stream& operator<<(roo_logging::Stream& os,
                                         const SensorBinding& binding);

  void sync() const {
    if (!synced_) {
      loc_ = store_.getSensorBinding(key_);
      synced_ = true;
    }
  }
  mutable SensorLocator loc_;

  BindingStore& store_;
  BindingStore::SensorKey key_;

  mutable bool synced_;
};

class BoundSensor {
 public:
  BoundSensor(Universe& universe, const SensorBinding* binding)
      : universe_(universe), binding_(binding) {}

  /// Reads the bound sensor or returns an initial measurement if unbound.
  Measurement read() const {
    SensorLocator loc = binding_->get();
    if (loc.isDefined()) {
      return universe_.read(loc);
    }
    return Measurement();
  }

 private:
  friend roo_logging::Stream& operator<<(roo_logging::Stream& os,
                                         const BoundSensor& sensor);

  Universe& universe_;
  const SensorBinding* binding_;
};

class ActuatorBinding {
 public:
  ActuatorBinding(BindingStore& store, BindingStore::ActuatorKey key)
      : loc_(), store_(store), key_(key), synced_(false) {}

  /// Returns the bound locator (empty if unbound).
  ActuatorLocator get() const {
    sync();
    return loc_;
  }

  /// Returns true when a locator is bound.
  bool isBound() const {
    sync();
    return loc_.isDefined();
  }

  /// Binds to the specified locator (or unbinds when undefined).
  void bind(const ActuatorLocator& loc) {
    if (loc_ == loc) return;
    loc_ = loc;
    if (!loc_.isDefined()) {
      store_.clearActuatorBinding(key_);
    } else {
      store_.setActuatorBinding(key_, loc_);
    }
    synced_ = true;
  }

  /// Clears the binding.
  void unbind() {
    loc_ = ActuatorLocator();
    store_.clearActuatorBinding(key_);
    synced_ = true;
  }

 private:
  friend roo_logging::Stream& operator<<(roo_logging::Stream& os,
                                         const ActuatorBinding& binding);

  void sync() const {
    if (!synced_) {
      loc_ = store_.getActuatorBinding(key_);
      synced_ = true;
    }
  }
  mutable ActuatorLocator loc_;

  BindingStore& store_;
  BindingStore::ActuatorKey key_;

  mutable bool synced_;
};

class BoundActuator {
 public:
  BoundActuator(Universe& universe, const ActuatorBinding* binding)
      : universe_(universe), binding_(binding) {}

  /// Writes to the bound actuator, if any.
  bool write(float value) const {
    ActuatorLocator loc = binding_->get();
    if (loc.isDefined()) {
      return universe_.write(loc, value);
    }
    return false;
  }

 private:
  friend roo_logging::Stream& operator<<(roo_logging::Stream& os,
                                         const BoundActuator& actuator);

  Universe& universe_;
  const ActuatorBinding* binding_;
};

/// Adapter for actuators that can be read, using actuator id as sensor id.
class BoundSensingActuator {
 public:
  BoundSensingActuator(Universe& universe, const ActuatorBinding* binding)
      : universe_(universe), binding_(binding) {}

  /// Reads from the bound actuator as a sensor.
  Measurement read() const {
    ActuatorLocator loc = binding_->get();
    if (loc.isDefined()) {
      SensorLocator sensor_loc(loc.device_locator(), loc.actuator_id());
      return universe_.read(sensor_loc);
    }
    return Measurement();
  }

  /// Writes to the bound actuator.
  bool write(float value) const {
    ActuatorLocator loc = binding_->get();
    if (loc.isDefined()) {
      return universe_.write(loc, value);
    }
    return false;
  }

 private:
  friend roo_logging::Stream& operator<<(roo_logging::Stream& os,
                                         const BoundSensingActuator& actuator);

  Universe& universe_;
  const ActuatorBinding* binding_;
};

}  // namespace roo_transceivers