#include "roo_transceivers/remote/server.h"

#include "roo_collections/flat_small_hash_set.h"
#include "roo_collections/hash.h"
#include "roo_logging.h"

#if !defined(MLOG_roo_transceivers_remote_server)
#define MLOG_roo_transceivers_remote_server 0
#endif

namespace roo_transceivers {

size_t DescriptorHashFn::operator()(
    const roo_transceivers_Descriptor& descriptor) const {
  size_t hash = 0;
  hash = roo_collections::murmur3_32(&descriptor.sensors_count,
                                     sizeof(descriptor.sensors_count), hash);
  hash = roo_collections::murmur3_32(&descriptor.actuators_count,
                                     sizeof(descriptor.actuators_count), hash);
  for (size_t i = 0; i < descriptor.sensors_count; ++i) {
    hash = roo_collections::murmur3_32(descriptor.sensors[i].id,
                                       strlen(descriptor.sensors[i].id), hash);
    hash = roo_collections::murmur3_32(&descriptor.sensors[i].quantity,
                                       sizeof(descriptor.sensors[i].quantity),
                                       hash);
  }
  for (size_t i = 0; i < descriptor.actuators_count; ++i) {
    hash = roo_collections::murmur3_32(
        descriptor.actuators[i].id, strlen(descriptor.actuators[i].id), hash);
    hash = roo_collections::murmur3_32(&descriptor.actuators[i].quantity,
                                       sizeof(descriptor.actuators[i].quantity),
                                       hash);
  }
  return hash;
}

UniverseServer::UniverseServer(Universe& universe,
                               UniverseServerChannel& channel,
                               Executor& transmit_executor)
    : universe_(universe),
      channel_(channel),
      transmit_executor_(transmit_executor),
      state_(),
      is_full_snapshot_(true),
      transmission_in_progress_(false),
      device_update_pending_(false),
      readings_pending_(false) {
  channel.registerClientMessageCallback(
      [this](const roo_transceivers_ClientMessage& msg) {
        handleClientMessage(msg);
      });
  universe_.addEventListener(this);
}

UniverseServer::~UniverseServer() {
  channel_.registerClientMessageCallback(nullptr);
  universe_.removeEventListener(this);
}

void UniverseServer::begin() { transmitInit(); }

void UniverseServer::devicesChanged() {
  {
    roo::lock_guard<roo::mutex> lock(state_guard_);
    if (transmission_in_progress_) {
      device_update_pending_ = true;
      return;
    }
    state_.clearDelta();
    snapshotDevices();
    snapshotSensorState();
    is_full_snapshot_ = false;
    transmission_in_progress_ = true;
  }
  triggerTransmission();
}

void UniverseServer::newReadingsAvailable() {
  {
    roo::lock_guard<roo::mutex> lock(state_guard_);
    if (transmission_in_progress_) {
      readings_pending_ = true;
      return;
    }
    state_.clearDelta();
    snapshotDevices();
    snapshotSensorState();

    is_full_snapshot_ = false;
    transmission_in_progress_ = true;
  }
  triggerTransmission();
}

void UniverseServer::handleClientMessage(
    const roo_transceivers_ClientMessage& msg) {
  switch (msg.which_contents) {
    case roo_transceivers_ClientMessage_request_update_tag: {
      MLOG(roo_transceivers_remote_server) << "Received request update";
      universe_.requestUpdate();
      break;
    }
    case roo_transceivers_ClientMessage_request_state_tag: {
      handleRequestState();
      break;
    }
    case roo_transceivers_ClientMessage_write_tag: {
      const auto& req = msg.contents.write;
      ActuatorLocator loc(req.device_locator_schema, req.device_locator_id,
                          req.device_locator_actuator_id);
      MLOG(roo_transceivers_remote_server)
          << "Received write request for " << loc << " with val " << req.value;
      universe_.write(loc, req.value);
      break;
    }
    default: {
      LOG(ERROR) << "Unexpected client message type " << msg.which_contents;
    }
  }
}

void UniverseServer::handleRequestState() {
  MLOG(roo_transceivers_remote_server) << "Received request state";
  {
    roo::lock_guard<roo::mutex> lock(state_guard_);
    if (transmission_in_progress_) {
      full_snapshot_requested_ = true;
      return;
    }
    state_.clearAll();
    snapshotDevices();
    snapshotSensorState();
    is_full_snapshot_ = true;
    transmission_in_progress_ = true;
  }
  triggerTransmission();
}

void UniverseServer::triggerTransmission() {
  transmit_executor_.execute([this]() { transmissionLoop(); });
}

void UniverseServer::State::clearAll() {
  clearDelta();
  devices_.clear();
  descriptors_.clear();
  descriptors_by_key_.clear();
  readings_.clear();
}

void UniverseServer::State::clearDelta() {
  device_deltas_.clear();
  descriptor_deltas_.clear();
  reading_delta_groups_.clear();
  reading_deltas_.clear();
}

void UniverseServer::State::newSensorReadingDelta(const SensorLocator& loc,
                                                  float value,
                                                  roo_time::Uptime time) {
  if (reading_delta_groups_.empty() ||
      loc.device_locator() != reading_delta_groups_.back().device) {
    reading_delta_groups_.push_back(
        SensorReadingDeltaDeviceGroup{loc.device_locator(), 1});
  } else {
    ++reading_delta_groups_.back().reading_count;
  }
  reading_deltas_.push_back(SensorReadingDelta{loc.sensor_id(), value, time});
}

void UniverseServer::snapshotDevices() {
  roo_transceivers_Descriptor descriptor;
  roo_collections::FlatSmallHashSet<DeviceLocator> removed(
      state_.device_count());
  for (const auto& itr : state_.devices()) {
    removed.insert(itr.first);
  }
  int ordinal = 0;
  universe_.forEachDevice([&](const DeviceLocator& loc) -> bool {
    if (!universe_.getDeviceDescriptor(loc, descriptor)) {
      LOG(WARNING) << "Found device without a descriptor, ignoring.";
      return true;
    }
    auto existing = state_.devices().find(loc);
    if (existing == state_.devices().end()) {
      // New device.
      state_.addDevice(loc, descriptor, ordinal);
    } else {
      // Device exists.
      removed.erase(loc);
      int old_descriptor_key = existing->second.descriptor_key;
      const roo_transceivers_Descriptor& old_descriptor =
          state_.descriptors_by_key()[old_descriptor_key];
      // Check if the descriptor changed.
      if (old_descriptor == descriptor) {
        state_.newDeviceDelta(loc, State::DeviceDelta::PRESERVED,
                              existing->second.ordinal);
        state_.addDeviceEntry(loc, ordinal, old_descriptor_key);
      } else {
        // Changed, indeed. Need to deref the old descriptor, and reference
        // the new one.
        state_.newDeviceDelta(loc, State::DeviceDelta::MODIFIED, -1);
        state_.removeReadings(loc, old_descriptor);
        state_.removeDescriptorReference(old_descriptor);
        int key = state_.addDescriptorReference(descriptor);
        state_.addDeviceEntry(loc, ordinal, key);
      }
    }
    ++ordinal;
    return true;
  });
  // Emit 'remove' entries for devices that we didn't see in the new snapshot.
  for (const auto& loc : removed) {
    state_.removeDevice(loc);
  }
  device_update_pending_ = false;
}

void UniverseServer::snapshotSensorState() {
  // We take devices from delta, because they are guaranteed to be the same as
  // the last enumeration from the universe, and ordered the same way. We just
  // need to skip the 'removed' ones.
  for (const auto& dev : state_.device_deltas()) {
    if (dev.status == State::DeviceDelta::REMOVED) continue;
    const roo_transceivers_Descriptor& descriptor =
        state_.getDescriptor(dev.locator);
    for (size_t i = 0; i < descriptor.sensors_count; i++) {
      SensorLocator sensor_locator(dev.locator, descriptor.sensors[i].id);
      Measurement measurement = universe_.read(sensor_locator);
      if (state_.updateSensorReading(sensor_locator, measurement)) {
        state_.newSensorReadingDelta(sensor_locator, measurement.value(),
                                     measurement.time());
      }
    }
  }
  readings_pending_ = false;
}

int UniverseServer::State::addDescriptorReference(
    const roo_transceivers_Descriptor& descriptor) {
  int key;
  bool is_new = false;
  auto itr = descriptors_.find(descriptor);
  if (itr != descriptors_.end()) {
    ++itr->second.refcount;
    key = itr->second.key;
  } else {
    // Not found; create a new entry.
    key = descriptors_.size();
    descriptors_[descriptor] = DescriptorEntry{.key = key, .refcount = 1};
    descriptors_by_key_[key] = descriptor;
    is_new = true;
  }
  if (is_new) {
    newDescriptorDelta(key, State::DescriptorDelta::ADDED);
  }
  return key;
}

void UniverseServer::State::removeDescriptorReference(
    const roo_transceivers_Descriptor& descriptor) {
  auto itr = descriptors_.find(descriptor);
  if (itr == descriptors_.end()) {
    LOG(ERROR) << "Descriptor not found when trying to remove reference";
    return;
  }
  if (--itr->second.refcount > 0) {
    return;
  }
  int key = itr->second.key;
  descriptors_.erase(descriptor);
  newDescriptorDelta(key, State::DescriptorDelta::REMOVED);
  descriptors_by_key_.erase(key);
}

void UniverseServer::transmitInit() {
  MLOG(roo_transceivers_remote_server) << "Transmitting Init";
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_init_tag;
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmitUpdateBegin(bool delta) {
  if (delta) {
    MLOG(roo_transceivers_remote_server) << "Transmitting Delta update begin";
  } else {
    MLOG(roo_transceivers_remote_server)
        << "Transmitting Full state update begin";
  }
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents =
      roo_transceivers_ServerMessage_transceiver_update_begin_tag;
  msg.contents.transceiver_update_begin.delta = delta;
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmitUpdateEnd() {
  MLOG(roo_transceivers_remote_server) << "Transmitting Update end";
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents =
      roo_transceivers_ServerMessage_transceiver_update_end_tag;
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmitDescriptorAdded(int key) {
  MLOG(roo_transceivers_remote_server) << "Transmitting Descriptor added";
  const roo_transceivers_Descriptor& descriptor =
      state_.descriptors_by_key()[key];
  {
    roo_transceivers_ServerMessage msg =
        roo_transceivers_ServerMessage_init_zero;
    msg.which_contents = roo_transceivers_ServerMessage_descriptor_added_tag;
    msg.contents.descriptor_added.key = key;
    msg.contents.descriptor_added.has_descriptor = true;
    msg.contents.descriptor_added.descriptor = descriptor;
    channel_.sendServerMessage(msg);
  }
}

void UniverseServer::transmitDescriptorRemoved(int key) {
  MLOG(roo_transceivers_remote_server) << "Transmitting Descriptor removed";
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_descriptor_removed_tag;
  msg.contents.descriptor_removed.key = key;
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmitDeviceAdded(const DeviceLocator& locator,
                                         int descriptor_key) {
  MLOG(roo_transceivers_remote_server) << "Transmitting Device added";
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_device_added_tag;
  auto& payload = msg.contents.device_added;
  strncpy(payload.locator_schema, locator.schema().c_str(), 16);
  strncpy(payload.locator_id, locator.device_id().c_str(), 24);
  msg.contents.device_added.descriptor_key = descriptor_key;
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmitDevicesPreserved(int first_preserved_ordinal,
                                              size_t count) {
  MLOG(roo_transceivers_remote_server)
      << "Transmitting Devices preserved (" << count << ")";
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_device_preserved_tag;
  auto& payload = msg.contents.device_preserved;
  payload.prev_index = first_preserved_ordinal;
  payload.has_count = (count > 1);
  if (payload.has_count) {
    payload.count = count;
  }
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmitDeviceModified(int prev_ordinal,
                                            int descriptor_key) {
  MLOG(roo_transceivers_remote_server)
      << "Transmitting Device modified at " << prev_ordinal;
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_device_modified_tag;
  auto& payload = msg.contents.device_modified;
  payload.prev_index = prev_ordinal;
  payload.descriptor_key = descriptor_key;
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmitDeviceRemoved(int prev_ordinal) {
  MLOG(roo_transceivers_remote_server)
      << "Transmitting Device removed at " << prev_ordinal;
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_device_removed_tag;
  auto& payload = msg.contents.device_removed;
  payload.prev_index = prev_ordinal;
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmitReadingsBegin() {
  // MLOG(roo_transceivers_remote_server) << "Transmitting readings begin";
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_readings_begin_tag;
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmitReadingsEnd() {
  // MLOG(roo_transceivers_remote_server) << "Transmitting readings end";
  roo_transceivers_ServerMessage msg = roo_transceivers_ServerMessage_init_zero;
  msg.which_contents = roo_transceivers_ServerMessage_readings_end_tag;
  channel_.sendServerMessage(msg);
}

void UniverseServer::transmissionLoop() {
  bool is_delta;
  {
    roo::lock_guard<roo::mutex> lock(state_guard_);
    CHECK(transmission_in_progress_);
    is_delta = !is_full_snapshot_;
    is_full_snapshot_ = false;
    if (MLOG_IS_ON(roo_transceivers_remote_server)) {
      MLOG(roo_transceivers_remote_server) << "Pre-transmit state: ";
      for (const auto& device : state_.devices()) {
        MLOG(roo_transceivers_remote_server)
            << "    " << device.first << ": (" << device.second.descriptor_key
            << ", " << device.second.ordinal << ")";
      }
    }
  }
  while (true) {
    MLOG(roo_transceivers_remote_server) << "Begin transmission";
    transmit(is_delta);
    {
      roo::lock_guard<roo::mutex> lock(state_guard_);
      CHECK(transmission_in_progress_);
      if (full_snapshot_requested_) {
        state_.clearAll();
        snapshotDevices();
        snapshotSensorState();
        is_full_snapshot_ = true;
        full_snapshot_requested_ = false;
      } else if (device_update_pending_) {
        state_.clearDelta();
        snapshotDevices();
        snapshotSensorState();
      } else if (readings_pending_) {
        state_.clearDelta();
        snapshotSensorState();
      } else {
        transmission_in_progress_ = false;
        return;
      }
      is_delta = !is_full_snapshot_;
    }
  }
  MLOG(roo_transceivers_remote_server) << "End transmission";
}

void UniverseServer::transmit(bool is_delta) {
  // Assumes that the flags have been checked under state_guard_ to authorize
  // the transmission.
  size_t delta_count = state_.device_deltas().size();
  if (delta_count > 0) {
    transmitUpdateBegin(is_delta);
    // Transmit new descriptors.
    for (const auto& itr : state_.descriptor_deltas()) {
      if (itr.status != State::DescriptorDelta::ADDED) continue;
      transmitDescriptorAdded(itr.key);
    }
    // Transmit all devices.
    size_t i = 0;
    while (i < delta_count) {
      const auto& delta = state_.device_deltas()[i];
      switch (delta.status) {
        case State::DeviceDelta::ADDED: {
          const auto& device = state_.devices()[delta.locator];
          transmitDeviceAdded(delta.locator, device.descriptor_key);
          ++i;
          break;
        }
        case State::DeviceDelta::PRESERVED: {
          int preserved_first_ordinal = delta.old_ordinal;
          size_t preserved_count = 1;
          while (i + 1 < delta_count) {
            const auto& next_delta = state_.device_deltas()[i + 1];
            // const auto& next_device = state_.devices()[next_delta.locator];
            if (next_delta.status != State::DeviceDelta::PRESERVED ||
                next_delta.old_ordinal != delta.old_ordinal + preserved_count) {
              break;
            }
            ++i;
            ++preserved_count;
          }
          transmitDevicesPreserved(preserved_first_ordinal, preserved_count);
          ++i;
          break;
        }
        case State::DeviceDelta::MODIFIED: {
          const auto& device = state_.devices()[delta.locator];
          transmitDeviceModified(device.ordinal, device.descriptor_key);
          ++i;
          break;
        }
        case State::DeviceDelta::REMOVED: {
          transmitDeviceRemoved(delta.old_ordinal);
          ++i;
          break;
        }
        default: {
          break;
        }
      }
    }
    // Transmit removed descriptors.
    for (const auto& itr : state_.descriptor_deltas()) {
      if (itr.status != State::DescriptorDelta::REMOVED) continue;
      transmitDescriptorRemoved(itr.key);
    }
    transmitUpdateEnd();
  }

  // Transmit new readings.
  if (!state_.reading_delta_groups().empty()) {
    transmitReadingsBegin();
    size_t reading_offset = 0;
    roo_time::Uptime now = roo_time::Uptime::Now();
    for (const auto& group : state_.reading_delta_groups()) {
      roo_transceivers_ServerMessage msg =
          roo_transceivers_ServerMessage_init_zero;
      msg.which_contents = roo_transceivers_ServerMessage_reading_tag;
      auto& payload = msg.contents.reading;
      strncpy(payload.device_locator_schema, group.device.schema().c_str(), 16);
      strncpy(payload.device_locator_id, group.device.device_id().c_str(), 24);
      payload.sensor_values_count = group.reading_count;
      for (size_t i = 0; i < group.reading_count; ++i) {
        auto& reading = state_.reading_deltas()[reading_offset];
        auto& dest = payload.sensor_values[i];
        strncpy(dest.device_locator_sensor_id, reading.sensor_id.c_str(), 24);
        dest.value = reading.value;
        dest.age_ms = (now - reading.time).inMillis();
        ++reading_offset;
      }
      channel_.sendServerMessage(msg);
    }
    transmitReadingsEnd();
  }
}

}  // namespace roo_transceivers