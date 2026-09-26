#include "environmental_sensor.h"

#include <cstring>

#include "roo_time.h"
#include "roo_transceivers/universe.h"

const roo_transceivers::Descriptor* getFakeEnvironmentalSensorDescriptor() {
  static roo_transceivers::Descriptor descriptor = [] {
    roo_transceivers::Descriptor result;
    for (const char* id : {"temperature"}) {
      auto* entry = result.add_sensors();
      entry->set_id(id);
      entry->set_quantity(roo_transceivers::Quantity::kTemperature);
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
    const roo_testing_transducers::Thermometer& thermometer) {
  return roo_transceivers::Measurement(roo_transceivers::Quantity::kTemperature,
                                       rounded_now(), thermometer.read().AsC());
}

}  // namespace

roo_transceivers::Measurement FakeEnvironmentalSensor::read(
    std::string_view sensor_id) const {
  if (sensor_id == "temperature") {
    return measurement(*thermometer_);
  } else {
    return roo_transceivers::Measurement();
  }
}
