#include "roo_transceivers/remote/proto.h"

#include "roo_logging.h"

namespace roo_transceivers {
namespace proto {

roo_transceivers::ServerMessage SrvInit() {
  roo_transceivers::ServerMessage msg = {};
  msg.mutable_init();
  return msg;
}

roo_transceivers::ServerMessage SrvFullUpdateBegin() {
  roo_transceivers::ServerMessage msg = {};
  msg.mutable_transceiver_update_begin()->set_delta(false);
  return msg;
}

roo_transceivers::ServerMessage SrvDeltaUpdateBegin() {
  roo_transceivers::ServerMessage msg = {};
  msg.mutable_transceiver_update_begin()->set_delta(true);
  return msg;
}

roo_transceivers::ServerMessage SrvUpdateEnd() {
  roo_transceivers::ServerMessage msg = {};
  msg.mutable_transceiver_update_end();
  return msg;
}

roo_transceivers::ServerMessage SrvDescriptorAdded(
    int key, const roo_transceivers::Descriptor& descriptor) {
  roo_transceivers::ServerMessage msg = {};
  msg.mutable_descriptor_added()->set_key(key);
  *msg.mutable_descriptor_added()->mutable_descriptor() = descriptor;
  return msg;
}

roo_transceivers::ServerMessage SrvDescriptorRemoved(int key) {
  roo_transceivers::ServerMessage msg = {};
  msg.mutable_descriptor_removed()->set_key(key);
  return msg;
}

roo_transceivers::ServerMessage SrvDeviceAdded(const DeviceLocator& locator,
                                               int descriptor_key) {
  roo_transceivers::ServerMessage msg = {};
  auto& payload = *msg.mutable_device_added();
  payload.set_locator_schema(locator.schema().c_str());
  payload.set_locator_id(locator.device_id().c_str());
  msg.mutable_device_added()->set_descriptor_key(descriptor_key);
  return msg;
}

roo_transceivers::ServerMessage SrvDevicesPreserved(int first_preserved_ordinal,
                                                    size_t count) {
  roo_transceivers::ServerMessage msg = {};
  auto& payload = *msg.mutable_device_preserved();
  payload.set_prev_index(first_preserved_ordinal);
  if (count > 1) {
    payload.set_count(count);
  }
  return msg;
}

roo_transceivers::ServerMessage SrvDevicesModified(int prev_ordinal,
                                                   int descriptor_key) {
  roo_transceivers::ServerMessage msg = {};
  auto& payload = *msg.mutable_device_modified();
  payload.set_prev_index(prev_ordinal);
  payload.set_descriptor_key(descriptor_key);
  return msg;
}

roo_transceivers::ServerMessage SrvDeviceRemoved(int prev_ordinal) {
  roo_transceivers::ServerMessage msg = {};
  auto& payload = *msg.mutable_device_removed();
  payload.set_prev_index(prev_ordinal);
  return msg;
}

roo_transceivers::ServerMessage SrvReadingsBegin() {
  roo_transceivers::ServerMessage msg = {};
  msg.mutable_readings_begin();
  return msg;
}

roo_transceivers::ServerMessage SrvReadingsEnd() {
  roo_transceivers::ServerMessage msg = {};
  msg.mutable_readings_end();
  return msg;
}

roo_transceivers::ServerMessage SrvReading(const DeviceLocator& device) {
  roo_transceivers::ServerMessage msg = {};
  auto& payload = *msg.mutable_reading();
  payload.set_device_locator_schema(device.schema().c_str());
  payload.set_device_locator_id(device.device_id().c_str());
  return msg;
}

void AddReading(roo_transceivers::ServerMessage& reading,
                const SensorId& sensor_id, float value, uint64_t age_ms) {
  CHECK(reading.has_reading());
  auto& payload = *reading.mutable_reading();
  auto& val = *payload.add_sensor_values();
  val.set_device_locator_sensor_id(sensor_id.c_str());
  val.set_value(value);
  val.set_age_ms(age_ms);
}

roo_transceivers::ClientMessage ClientRequestUpdate() {
  roo_transceivers::ClientMessage msg = {};
  msg.mutable_request_update();
  return msg;
}

roo_transceivers::ClientMessage ClientRequestState() {
  roo_transceivers::ClientMessage msg = {};
  msg.mutable_request_state();
  return msg;
}

roo_transceivers::ClientMessage ClientWrite(const ActuatorLocator& actuator,
                                            float value) {
  roo_transceivers::ClientMessage msg = {};
  auto& payload = *msg.mutable_write();
  payload.set_device_locator_schema(actuator.schema().c_str());
  payload.set_device_locator_id(actuator.device_id().c_str());
  payload.set_device_locator_actuator_id(actuator.actuator_id().c_str());
  payload.set_value(value);
  return msg;
}

}  // namespace proto

}  // namespace roo_transceivers
