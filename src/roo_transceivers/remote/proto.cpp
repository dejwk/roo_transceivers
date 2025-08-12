#include "roo_transceivers/remote/proto.h"

#include "roo_logging.h"

namespace roo_transceivers {
namespace proto {

roo_transceivers_ServerMessage SrvInit() {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_init_tag;
  return msg;
}

roo_transceivers_ServerMessage SrvFullUpdateBegin() {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents =
      roo_transceivers_ServerMessage_transceiver_update_begin_tag;
  msg.contents.transceiver_update_begin.delta = false;
  return msg;
}

roo_transceivers_ServerMessage SrvDeltaUpdateBegin() {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents =
      roo_transceivers_ServerMessage_transceiver_update_begin_tag;
  msg.contents.transceiver_update_begin.delta = true;
  return msg;
}

roo_transceivers_ServerMessage SrvUpdateEnd() {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents =
      roo_transceivers_ServerMessage_transceiver_update_end_tag;
  return msg;
}

roo_transceivers_ServerMessage SrvDescriptorAdded(
    int key, const roo_transceivers_Descriptor& descriptor) {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_descriptor_added_tag;
  msg.contents.descriptor_added.key = key;
  msg.contents.descriptor_added.has_descriptor = true;
  msg.contents.descriptor_added.descriptor = descriptor;
  return msg;
}

roo_transceivers_ServerMessage SrvDescriptorRemoved(int key) {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_descriptor_removed_tag;
  msg.contents.descriptor_removed.key = key;
  return msg;
}

roo_transceivers_ServerMessage SrvDeviceAdded(const DeviceLocator& locator,
                                              int descriptor_key) {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_device_added_tag;
  auto& payload = msg.contents.device_added;
  strncpy(payload.locator_schema, locator.schema().c_str(),
          DeviceSchema::kCapacity);
  strncpy(payload.locator_id, locator.device_id().c_str(), DeviceId::kCapacity);
  msg.contents.device_added.descriptor_key = descriptor_key;
  return msg;
}

roo_transceivers_ServerMessage SrvDevicesPreserved(int first_preserved_ordinal,
                                                   size_t count) {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_device_preserved_tag;
  auto& payload = msg.contents.device_preserved;
  payload.prev_index = first_preserved_ordinal;
  payload.has_count = (count > 1);
  if (payload.has_count) {
    payload.count = count;
  }
  return msg;
}

roo_transceivers_ServerMessage SrvDevicesModified(int prev_ordinal,
                                                  int descriptor_key) {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_device_modified_tag;
  auto& payload = msg.contents.device_modified;
  payload.prev_index = prev_ordinal;
  payload.descriptor_key = descriptor_key;
  return msg;
}

roo_transceivers_ServerMessage SrvDevicesRemoved(int prev_ordinal) {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_device_removed_tag;
  auto& payload = msg.contents.device_removed;
  payload.prev_index = prev_ordinal;
  return msg;
}

roo_transceivers_ServerMessage SrvReadingsBegin() {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_readings_begin_tag;
  return msg;
}

roo_transceivers_ServerMessage SrvReadingsEnd() {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_readings_end_tag;
  return msg;
}

roo_transceivers_ServerMessage SrvReading(const DeviceLocator& device) {
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_reading_tag;
  auto& payload = msg.contents.reading;
  strncpy(payload.device_locator_schema, device.schema().c_str(),
          DeviceSchema::kCapacity);
  strncpy(payload.device_locator_id, device.device_id().c_str(),
          DeviceId::kCapacity);
  return msg;
}

void AddReading(roo_transceivers_ServerMessage& reading,
                const SensorId& sensor_id, float value, uint64_t age_ms) {
  //   CHECK_EQ(reading.which_contents,
  //   roo_transceivers_ServerMessage_reading_tag);
  auto& payload = reading.contents.reading;
  //   CHECK_LT(payload.sensor_values_count, 16);
  auto& val = payload.sensor_values[payload.sensor_values_count];
  strncpy(val.device_locator_sensor_id, sensor_id.c_str(), SensorId::kCapacity);
  val.value = value;
  val.age_ms = age_ms;
  ++payload.sensor_values_count;
}

roo_transceivers_ClientMessage ClientRequestUpdate() {
  roo_transceivers_ClientMessage msg = roo_transceivers_ClientMessage_init_zero;
  msg.which_contents = roo_transceivers_ClientMessage_request_update_tag;
  return msg;
}

roo_transceivers_ClientMessage ClientRequestState() {
  roo_transceivers_ClientMessage msg = roo_transceivers_ClientMessage_init_zero;
  msg.which_contents = roo_transceivers_ClientMessage_request_state_tag;
  return msg;
}

roo_transceivers_ClientMessage ClientWrite(const ActuatorLocator& actuator,
                                           float value) {
  roo_transceivers_ClientMessage msg = roo_transceivers_ClientMessage_init_zero;
  msg.which_contents = roo_transceivers_ClientMessage_write_tag;
  auto& payload = msg.contents.write;
  strncpy(payload.device_locator_schema, actuator.schema().c_str(),
          DeviceSchema::kCapacity);
  strncpy(payload.device_locator_id, actuator.device_id().c_str(),
          DeviceId::kCapacity);
  strncpy(payload.device_locator_actuator_id, actuator.actuator_id().c_str(),
          ActuatorId::kCapacity);
  payload.value = value;
  return msg;
}

}  // namespace proto

}  // namespace roo_transceivers