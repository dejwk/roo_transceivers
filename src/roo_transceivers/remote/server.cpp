#include "roo_transceivers/remote/server.h"

#include "roo_collections/flat_small_hash_set.h"
#include "roo_collections/hash.h"
#include "roo_logging.h"
#include "roo_transceivers/remote/proto.h"

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
      full_snapshot_transmitted_(false),
      transmission_in_progress_(false),
      state_snapshot_pending_(false),
      device_update_pending_(false),
      readings_pending_(false),
      devices_changed_(false) {
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
  bool send_full_state;
  {
    roo::lock_guard<roo::mutex> lock(state_guard_);
    if (transmission_in_progress_) {
      device_update_pending_ = true;
      return;
    }
    state_.clearDelta();
    snapshotDevices();
    snapshotSensorState(true);
    transmission_in_progress_ = true;
    send_full_state = !full_snapshot_transmitted_;
  }
  triggerTransmission(send_full_state);
}

void UniverseServer::newReadingsAvailable() {
  bool send_full_state;
  {
    roo::lock_guard<roo::mutex> lock(state_guard_);
    if (transmission_in_progress_) {
      readings_pending_ = true;
      return;
    }
    state_.clearDelta();
    snapshotDevices();
    snapshotSensorState(false);
    transmission_in_progress_ = true;
    send_full_state = !full_snapshot_transmitted_;
  }
  triggerTransmission(send_full_state);
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
      state_snapshot_pending_ = true;
      return;
    }
    state_.clearAll();
    snapshotDevices();
    snapshotSensorState(false);
    transmission_in_progress_ = true;
  }
  triggerTransmission(true);
}

void UniverseServer::triggerTransmission(bool send_full_snapshot) {
  transmit_executor_.execute(
      [this, send_full_snapshot]() { transmissionLoop(send_full_snapshot); });
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
  devices_changed_ = false;
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
      devices_changed_ = true;
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
        devices_changed_ = true;
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
    devices_changed_ = true;
    state_.removeDevice(loc);
  }
  device_update_pending_ = false;
}

void UniverseServer::snapshotSensorState(bool new_only) {
  // We take devices from delta, because they are guaranteed to be the same as
  // the last enumeration from the universe, and ordered the same way. We just
  // need to skip the 'removed' ones.
  for (const auto& dev : state_.device_deltas()) {
    if (dev.status == State::DeviceDelta::REMOVED) continue;
    if (new_only && dev.status != State::DeviceDelta::ADDED) continue;
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
    key = next_descriptor_key_++;
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
  channel_.sendServerMessage(proto::SrvInit());
}

void UniverseServer::transmitUpdateBegin(bool delta) {
  if (delta) {
    MLOG(roo_transceivers_remote_server) << "Transmitting Delta update begin";
    channel_.sendServerMessage(proto::SrvDeltaUpdateBegin());
  } else {
    MLOG(roo_transceivers_remote_server)
        << "Transmitting Full state update begin";
    channel_.sendServerMessage(proto::SrvFullUpdateBegin());
  }
}

void UniverseServer::transmitUpdateEnd() {
  MLOG(roo_transceivers_remote_server) << "Transmitting Update end";
  channel_.sendServerMessage(proto::SrvUpdateEnd());
}

void UniverseServer::transmitDescriptorAdded(int key) {
  MLOG(roo_transceivers_remote_server) << "Transmitting Descriptor added";
  const roo_transceivers_Descriptor& descriptor =
      state_.descriptors_by_key()[key];
  channel_.sendServerMessage(proto::SrvDescriptorAdded(key, descriptor));
}

void UniverseServer::transmitDescriptorRemoved(int key) {
  MLOG(roo_transceivers_remote_server) << "Transmitting Descriptor removed";
  channel_.sendServerMessage(proto::SrvDescriptorRemoved(key));
}

void UniverseServer::transmitDeviceAdded(const DeviceLocator& locator,
                                         int descriptor_key) {
  MLOG(roo_transceivers_remote_server) << "Transmitting Device added";
  channel_.sendServerMessage(proto::SrvDeviceAdded(locator, descriptor_key));
}

void UniverseServer::transmitDevicesPreserved(int first_preserved_ordinal,
                                              size_t count) {
  MLOG(roo_transceivers_remote_server)
      << "Transmitting Devices preserved (" << count << ")";
  channel_.sendServerMessage(
      proto::SrvDevicesPreserved(first_preserved_ordinal, count));
}

void UniverseServer::transmitDeviceModified(int prev_ordinal,
                                            int descriptor_key) {
  MLOG(roo_transceivers_remote_server)
      << "Transmitting Device modified at " << prev_ordinal;
  channel_.sendServerMessage(
      proto::SrvDevicesModified(prev_ordinal, descriptor_key));
}

void UniverseServer::transmitDeviceRemoved(int prev_ordinal) {
  MLOG(roo_transceivers_remote_server)
      << "Transmitting Device removed at " << prev_ordinal;
  channel_.sendServerMessage(proto::SrvDeviceRemoved(prev_ordinal));
}

void UniverseServer::transmitReadingsBegin() {
  MLOG(roo_transceivers_remote_server) << "Transmitting readings begin";
  channel_.sendServerMessage(proto::SrvReadingsBegin());
}

void UniverseServer::transmitReadingsEnd() {
  MLOG(roo_transceivers_remote_server) << "Transmitting readings end";
  channel_.sendServerMessage(proto::SrvReadingsEnd());
}

void UniverseServer::transmissionLoop(bool send_full_snapshot) {
  bool is_delta;
  {
    roo::lock_guard<roo::mutex> lock(state_guard_);
    CHECK(transmission_in_progress_);
    is_delta = !send_full_snapshot;
    send_full_snapshot = false;
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
      if (!is_delta) {
        full_snapshot_transmitted_ = true;
      }
      if (state_snapshot_pending_) {
        state_.clearAll();
        snapshotDevices();
        snapshotSensorState(false);
        send_full_snapshot = true;
        state_snapshot_pending_ = false;
      } else if (device_update_pending_) {
        state_.clearDelta();
        snapshotDevices();
        snapshotSensorState(true);
      } else if (readings_pending_) {
        state_.clearDelta();
        snapshotSensorState(false);
      } else {
        transmission_in_progress_ = false;
        return;
      }
      is_delta = !send_full_snapshot;
    }
  }
  MLOG(roo_transceivers_remote_server) << "End transmission";
}

void UniverseServer::transmit(bool is_delta) {
  // Assumes that the flags have been checked under state_guard_ to authorize
  // the transmission.
  if (devices_changed_) {
    transmitUpdateBegin(is_delta);
    // Transmit new descriptors.
    for (const auto& itr : state_.descriptor_deltas()) {
      if (itr.status != State::DescriptorDelta::ADDED) continue;
      transmitDescriptorAdded(itr.key);
    }
    // Transmit all devices.
    size_t i = 0;
    size_t delta_count = state_.device_deltas().size();
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
      roo_transceivers_ServerMessage msg = proto::SrvReading(group.device);
      for (size_t i = 0; i < group.reading_count; ++i) {
        auto& reading = state_.reading_deltas()[reading_offset];
        proto::AddReading(msg, reading.sensor_id, reading.value,
                          (now - reading.time).inMillis());
        ++reading_offset;
      }
      channel_.sendServerMessage(msg);
    }
    transmitReadingsEnd();
  }
}

}  // namespace roo_transceivers