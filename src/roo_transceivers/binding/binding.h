#pragma once

#include <cstdint>

#include "roo_logging.h"
#include "roo_transceivers/binding/hal/store.h"
#include "roo_transceivers/measurement.h"
#include "roo_transceivers/universe.h"

namespace roo_transceivers {

class SensorBinding {
 public:
  SensorBinding(BindingStore& store, BindingStore::SensorKey key)
      : loc_(), store_(store), key_(key), synced_(false) {}

  SensorLocator get() const {
    sync();
    return loc_;
  }

  bool isBound() const {
    sync();
    return loc_.isDefined();
  }

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

  Measurement read() const {
    SensorLocator loc = binding_->get();
    if (loc.isDefined()) {
      return universe_.read(binding_->get());
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

  ActuatorLocator get() const {
    sync();
    return loc_;
  }

  bool isBound() const {
    sync();
    return loc_.isDefined();
  }

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

  bool write(float value) const {
    ActuatorLocator loc = binding_->get();
    if (loc.isDefined()) {
      return universe_.write(binding_->get(), value);
    }
    return false;
  }

 private:
  friend roo_logging::Stream& operator<<(roo_logging::Stream& os,
                                         const BoundActuator& actuator);

  Universe& universe_;
  const ActuatorBinding* binding_;
};

// For actuators that can be read, using the actuator_id as sensor_id.
class BoundSensingActuator {
 public:
  BoundSensingActuator(Universe& universe, const ActuatorBinding* binding)
      : universe_(universe), binding_(binding) {}

  Measurement read() const {
    ActuatorLocator loc = binding_->get();
    if (loc.isDefined()) {
      SensorLocator sensor_loc(loc.device_locator(), loc.actuator_id());
      return universe_.read(sensor_loc);
    }
    return Measurement();
  }

  bool write(float value) const {
    ActuatorLocator loc = binding_->get();
    if (loc.isDefined()) {
      return universe_.write(binding_->get(), value);
    }
    return false;
  }

 private:
  friend roo_logging::Stream& operator<<(roo_logging::Stream& os,
                                         const BoundActuator& actuator);

  Universe& universe_;
  const ActuatorBinding* binding_;
};

}  // namespace roo_transceivers