#include "relay.h"

#include <cstring>

#include "roo_time.h"
#include "roo_transceivers/universe.h"

roo_transceivers::Descriptor* getFakeRooRelay4pDescriptor() {
  static roo_transceivers::Descriptor descriptor = [] {
    roo_transceivers::Descriptor result;
    for (const char* id : {"relay1", "relay2", "relay3", "relay4"}) {
      auto* entry = result.add_sensors();
      entry->set_id(id);
      entry->set_quantity(roo_transceivers::Quantity::kBinaryState);
    }
    for (const char* id : {"relay1", "relay2", "relay3", "relay4"}) {
      auto* entry = result.add_actuators();
      entry->set_id(id);
      entry->set_quantity(roo_transceivers::Quantity::kBinaryState);
    }
    return result;
  }();
  return &descriptor;
}

namespace {

roo_time::Uptime rounded_now() {
  return roo_time::Uptime::Start() +
         roo_time::Seconds(
             (roo_time::Uptime::Now() - roo_time::Uptime::Start()).inSeconds());
}

roo_transceivers::Measurement measurement(
    roo_testing_transducers::DigitalLevel level) {
  return roo_transceivers::Measurement(
      roo_transceivers::Quantity::kBinaryState, rounded_now(),
      level == roo_testing_transducers::kDigitalHigh ? 1.0f : 0.0f);
}

}  // namespace

roo_transceivers::Measurement FakeRooRelay4p::read(
    std::string_view sensor_id) const {
  if (sensor_id == "relay1") {
    return measurement(levels_[0]);
  } else if (sensor_id == "relay2") {
    return measurement(levels_[1]);
  } else if (sensor_id == "relay3") {
    return measurement(levels_[2]);
  } else if (sensor_id == "relay4") {
    return measurement(levels_[3]);
  } else {
    return roo_transceivers::Measurement();
  }
}

bool FakeRooRelay4p::write(std::string_view actuator_id, float value) {
  roo_testing_transducers::DigitalLevel level;
  if (value == 1.0f) {
    level = roo_testing_transducers::kDigitalHigh;
  } else if (value == 0.0f) {
    level = roo_testing_transducers::kDigitalLow;
  } else {
    return false;
  }
  if (actuator_id == "relay1") {
    levels_[0] = level;
    write_fn_(0, *this, level);
  } else if (actuator_id == "relay2") {
    levels_[1] = level;
    write_fn_(1, *this, level);
  } else if (actuator_id == "relay3") {
    levels_[2] = level;
    write_fn_(2, *this, level);
  } else if (actuator_id == "relay4") {
    levels_[3] = level;
    write_fn_(3, *this, level);
  } else {
    return false;
  }
  return true;
}
