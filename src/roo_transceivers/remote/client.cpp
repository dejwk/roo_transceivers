#include "roo_transceivers/remote/client.h"

#include "roo_logging.h"
#include "roo_transceivers/remote/proto.h"

#if !defined(MLOG_roo_transceivers_remote_client)
#define MLOG_roo_transceivers_remote_client 0
#endif

namespace roo_transceivers {

UniverseClient::UniverseClient(UniverseClientChannel& channel)
    : channel_(channel), synced_(false) {
  channel.registerServerMessageCallback(
      [this](const roo_transceivers_ServerMessage& msg) {
        handleServerMessage(msg);
      });
}

UniverseClient::~UniverseClient() {
  channel_.registerServerMessageCallback(nullptr);
}

void UniverseClient::begin() {
  channel_.sendClientMessage(proto::ClientRequestState());
}

size_t UniverseClient::deviceCount() const {
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  return devices_.size();
}

bool UniverseClient::forEachDevice(
    std::function<bool(const DeviceLocator&)> callback) const {
  MLOG(roo_transceivers_remote_client) << "Enumerating devices ...";
  size_t i = 0;
  DeviceLocator loc;

  bool result = true;
  while (true) {
    {
      // Can't hold mutex for the entire iteration, b/c of deadlocks, when the
      // callback tries to call getDescriptor() or something else that grabs the
      // mutex.
      const roo::lock_guard<roo::mutex> lock(state_guard_);
      if (i >= devices_.size()) break;
      loc = devices_[i].locator;
    }
    if (!callback(loc)) {
      result = false;
      break;
    }
    ++i;
  }
  MLOG(roo_transceivers_remote_client) << "Enumerating devices done.";
  return result;
}

const roo_transceivers_Descriptor* UniverseClient::lookupDeviceDescriptor(
    const DeviceLocator& locator, int& descriptor_key) const {
  auto device_itr = device_idx_by_locator_.find(locator);
  if (device_itr == device_idx_by_locator_.end()) {
    // Device with the specified locator has not been found.
    return nullptr;
  }
  descriptor_key = devices_[device_itr->second].descriptor_key;
  auto descriptor_itr = descriptors_.find(descriptor_key);
  if (descriptor_itr == descriptors_.end()) {
    // Descriptor for the specified device is (erroneously) missing.
    LOG(WARNING) << "No descriptor for device " << locator;
    return nullptr;
  }
  return &descriptor_itr->second;
}

bool UniverseClient::getDeviceDescriptor(
    const DeviceLocator& locator,
    roo_transceivers_Descriptor& descriptor) const {
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  int descriptor_key;
  const auto* result = lookupDeviceDescriptor(locator, descriptor_key);
  if (result == nullptr) return false;
  descriptor = *result;
  return true;
}

Measurement UniverseClient::read(const SensorLocator& locator) const {
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  int descriptor_key;
  const auto* descriptor =
      lookupDeviceDescriptor(locator.device_locator(), descriptor_key);
  if (descriptor == nullptr) return Measurement();
  auto itr = readings_.find(locator);
  if (itr == readings_.end()) {
    return Measurement();
  }
  return itr->second;
}

bool UniverseClient::write(const ActuatorLocator& locator, float value) {
  {
    const roo::lock_guard<roo::mutex> lock(state_guard_);

    int descriptor_key;
    const auto* descriptor =
        lookupDeviceDescriptor(locator.device_locator(), descriptor_key);
    if (descriptor == nullptr) {
      // LOG(WARNING) << "Attempt to write to an unknown device " <<
      // locator.device_locator();
      return false;
    }
    if (!actuators_.contains(locator)) {
      LOG(WARNING) << "Attempt to write to an unknown actuator " << locator;
      return false;
    }
  }
  channel_.sendClientMessage(proto::ClientWrite(locator, value));
  return true;
}

void UniverseClient::requestUpdate() {
  channel_.sendClientMessage(proto::ClientRequestUpdate());
}

void UniverseClient::addEventListener(EventListener* listener) {
  const roo::lock_guard<roo::mutex> lock(listener_guard_);
  listeners_.insert(listener);
}

void UniverseClient::removeEventListener(EventListener* listener) {
  const roo::lock_guard<roo::mutex> lock(listener_guard_);
  listeners_.erase(listener);
}

void UniverseClient::notifyDevicesChanged() {
  const roo::lock_guard<roo::mutex> lock(listener_guard_);
  for (auto* listener : listeners_) {
    listener->devicesChanged();
  }
}

void UniverseClient::notifyReadingsAvailable() {
  const roo::lock_guard<roo::mutex> lock(listener_guard_);
  for (auto* listener : listeners_) {
    listener->newReadingsAvailable();
  }
}

void UniverseClient::handleServerMessage(
    const roo_transceivers_ServerMessage& msg) {
  switch (msg.which_contents) {
    case roo_transceivers_ServerMessage_init_tag: {
      handleInit();
      break;
    }
    case roo_transceivers_ServerMessage_transceiver_update_begin_tag: {
      handleUpdateBegin(msg.contents.transceiver_update_begin.delta);
      break;
    }
    case roo_transceivers_ServerMessage_descriptor_added_tag: {
      handleDescriptorAdded(msg.contents.descriptor_added.key,
                            msg.contents.descriptor_added.descriptor);
      break;
    }
    case roo_transceivers_ServerMessage_descriptor_removed_tag: {
      handleDescriptorRemoved(msg.contents.descriptor_removed.key);
      break;
    }
    case roo_transceivers_ServerMessage_device_added_tag: {
      DeviceLocator loc(msg.contents.device_added.locator_schema,
                        msg.contents.device_added.locator_id);
      handleDeviceAdded(loc, msg.contents.device_added.descriptor_key);
      break;
    }
    case roo_transceivers_ServerMessage_device_removed_tag: {
      handleDeviceRemoved(msg.contents.device_removed.prev_index);
      break;
    }
    case roo_transceivers_ServerMessage_device_preserved_tag: {
      size_t count = 1;
      const auto& payload = msg.contents.device_preserved;
      if (payload.has_count) {
        count = payload.count;
      }
      handleDevicePreserved(payload.prev_index, count);
      break;
    }
    case roo_transceivers_ServerMessage_device_modified_tag: {
      handleDeviceModified(msg.contents.device_modified.prev_index,
                           msg.contents.device_modified.descriptor_key);
      break;
    }
    case roo_transceivers_ServerMessage_transceiver_update_end_tag: {
      handleUpdateEnd();
      break;
    }
    case roo_transceivers_ServerMessage_readings_begin_tag: {
      handleReadingsBegin();
      break;
    }
    case roo_transceivers_ServerMessage_reading_tag: {
      auto& payload = msg.contents.reading;
      DeviceLocator device(payload.device_locator_schema,
                           payload.device_locator_id);
      handleReadings(device, payload.sensor_values,
                     payload.sensor_values_count);
      break;
    }
    case roo_transceivers_ServerMessage_readings_end_tag: {
      handleReadingsEnd();
      break;
    }

    default: {
      LOG(WARNING) << "Unexpected server message " << msg.which_contents;
      break;
    }
  }
}

void UniverseClient::handleInit() {
  {
    // Cancel the update, if any pending.
    roo::lock_guard<roo::mutex> lock(state_guard_);
    updated_devices_.clear();
    synced_ = false;
  }
  channel_.sendClientMessage(proto::ClientRequestState());
}

void UniverseClient::handleUpdateBegin(bool delta) {
  roo::lock_guard<roo::mutex> lock(state_guard_);
  if (!delta) {
    clearAll();
    synced_ = true;
    MLOG(roo_transceivers_remote_client)
        << "Received full update begin message";
  } else {
    MLOG(roo_transceivers_remote_client)
        << "Received delta update begin message";
  }
  CHECK(updated_devices_.empty());
}

void UniverseClient::handleUpdateEnd() {
  {
    roo::lock_guard<roo::mutex> lock(state_guard_);
    if (!synced_) return;
    devices_.swap(updated_devices_);
    updated_devices_.clear();
    device_idx_by_locator_.clear();
    for (size_t i = 0; i < devices_.size(); ++i) {
      device_idx_by_locator_[devices_[i].locator] = i;
    }
  }
  if (MLOG_IS_ON(roo_transceivers_remote_client)) {
    MLOG(roo_transceivers_remote_client) << "Post-receive state: ";
    for (const auto& device : devices_) {
      MLOG(roo_transceivers_remote_client)
          << "    " << device.locator << ": " << device.descriptor_key;
    }
  }
  notifyDevicesChanged();
}

void UniverseClient::handleDescriptorAdded(
    int key, const roo_transceivers_Descriptor& descriptor) {
  MLOG(roo_transceivers_remote_client) << "Received added descriptor";
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  if (!synced_) return;
  descriptors_[key] = descriptor;
}

void UniverseClient::handleDescriptorRemoved(int key) {
  MLOG(roo_transceivers_remote_client) << "Received removed descriptor";
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  if (!synced_) return;
  // At this point we do not expect to have any devices pointing to this
  // descriptor.
  descriptors_.erase(key);
}

void UniverseClient::handleDeviceAdded(const DeviceLocator& locator,
                                       int descriptor_key) {
  MLOG(roo_transceivers_remote_client) << "Received added device " << locator;
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  if (!synced_) return;
  auto itr = descriptors_.find(descriptor_key);
  if (itr == descriptors_.end()) {
    LOG(WARNING)
        << "Bogus server message (DeviceAdded): unknown descriptor key "
        << descriptor_key;
    return;
  }
  updated_devices_.push_back(DeviceEntry{locator, descriptor_key});
  const roo_transceivers_Descriptor& descriptor = itr->second;
  // Pre-initialize all sensor readings to set quantities.
  for (size_t i = 0; i < descriptor.sensors_count; ++i) {
    SensorLocator sensor_locator(locator, descriptor.sensors[i].id);
    readings_[sensor_locator] =
        Measurement(descriptor.sensors[i].quantity, roo_time::Uptime::Start());
  }
  // Also, register the actuators for fast lookup during write().
  for (size_t i = 0; i < descriptor.actuators_count; ++i) {
    ActuatorLocator actuator_locator(locator, descriptor.actuators[i].id);
    actuators_.insert(actuator_locator);
  }
}

void UniverseClient::handleDeviceRemoved(int prev_index) {
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  if (!synced_) return;
  int descriptor_key;
  if (prev_index < 0 || prev_index >= devices_.size()) {
    LOG(WARNING) << "Bogus server message (DeviceRemoved): prev_index of "
                 << prev_index << " is out of bounds; device count is "
                 << devices_.size();
    return;
  }
  const DeviceLocator& locator = devices_[prev_index].locator;
  MLOG(roo_transceivers_remote_client) << "Received removed device " << locator;
  // Erase all readings.
  const roo_transceivers_Descriptor* descriptor =
      lookupDeviceDescriptor(locator, descriptor_key);
  if (descriptor == nullptr) {
    LOG(WARNING) << "Bogus server message (DeviceRemoved): missing device "
                    "descriptor for "
                 << locator;
  } else {
    for (size_t i = 0; i < descriptor->sensors_count; ++i) {
      SensorLocator sensor_locator(locator, descriptor->sensors[i].id);
      readings_.erase(sensor_locator);
    }
    for (size_t i = 0; i < descriptor->actuators_count; ++i) {
      ActuatorLocator actuator_locator(locator, descriptor->actuators[i].id);
      actuators_.erase(actuator_locator);
    }
  }
  // That's it - we're not adding anything to updated_devices_, and
  // device_idx_by_locator will be refreshed on transceiver_update_end.
}

void UniverseClient::handleDevicePreserved(int prev_index_first, size_t count) {
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  if (!synced_) return;
  MLOG(roo_transceivers_remote_client)
      << "Received preserved devices (" << count << " at " << prev_index_first
      << ")";
  if (prev_index_first < 0 || prev_index_first + count > devices_.size()) {
    LOG(WARNING) << "Bogus server message (DevicePreserved): the range ("
                 << prev_index_first << ", " << prev_index_first + count
                 << ") is out of bounds; device count is " << devices_.size();
    return;
  }
  for (size_t i = 0; i < count; ++i) {
    updated_devices_.push_back(devices_[prev_index_first + i]);
  }
}

void UniverseClient::handleDeviceModified(int prev_index, int descriptor_key) {
  MLOG(roo_transceivers_remote_client)
      << "Received modified device at " << prev_index;
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  if (!synced_) return;
  if (prev_index < 0 || prev_index >= devices_.size()) {
    LOG(WARNING) << "Bogus server message (DeviceModified): prev_index of "
                 << prev_index << " is out of bounds; device count is "
                 << devices_.size();
    return;
  }
  updated_devices_.push_back(
      DeviceEntry{devices_[prev_index].locator, descriptor_key});
}

void UniverseClient::clearAll() {
  descriptors_.clear();
  devices_.clear();
  updated_devices_.clear();
  device_idx_by_locator_.clear();
  readings_.clear();
  actuators_.clear();
}

void UniverseClient::handleReadings(
    const DeviceLocator& device,
    const roo_transceivers_ServerMessage_Reading_SensorValue* readings,
    size_t readings_count) {
  const roo::lock_guard<roo::mutex> lock(state_guard_);
  if (!synced_) return;
  int descriptor_key;
  const roo_transceivers_Descriptor* descriptor =
      lookupDeviceDescriptor(device, descriptor_key);
  if (descriptor == nullptr) {
    LOG(WARNING)
        << "Bogus server message (Readings): missing device descriptor for "
        << device;
    return;
  }
  roo_time::Uptime now = roo_time::Uptime::Now();
  for (size_t i = 0; i < readings_count; ++i) {
    SensorLocator sensor_locator(device, readings[i].device_locator_sensor_id);
    Measurement& m = readings_[sensor_locator];
    // Overwrite the measurement with new value and time (but keep the
    // quantity).
    MLOG(roo_transceivers_remote_client)
        << "Received reading of " << sensor_locator << ": "
        << readings[i].value;
    m = Measurement(m.quantity(), now - roo_time::Millis(readings[i].age_ms),
                    readings[i].value);
  }
}

void UniverseClient::handleReadingsBegin() {}

void UniverseClient::handleReadingsEnd() { notifyReadingsAvailable(); }

}  // namespace roo_transceivers