#pragma once

#include <functional>
#include <vector>

#include "roo_threads/mutex.h"
#include "roo_transceivers.h"

namespace roo_transceivers {

class Executor {
 public:
  virtual ~Executor() = default;

  virtual void execute(std::function<void()> task) = 0;
};

struct DescriptorHashFn {
  size_t operator()(const roo_transceivers_Descriptor& descriptor) const;
};

struct DescriptorEntry {
  int key;
  int refcount;
};

using DescriptorMap =
    roo_collections::FlatSmallHashMap<roo_transceivers_Descriptor,
                                      DescriptorEntry, DescriptorHashFn>;

class UniverseServerChannel {
 public:
  using ClientMessageCb =
      std::function<void(const roo_transceivers_ClientMessage&)>;

  virtual ~UniverseServerChannel() = default;

  virtual void registerClientMessageCallback(ClientMessageCb cb) = 0;

  virtual void sendServerMessage(const roo_transceivers_ServerMessage& msg) = 0;
};

// The server keeps the cached state of the underlying universe, along with the
// most recent delta. It also has state, which indicates whether a transmission
// is currently in progress, and whether the universe state has changed since
// the last snapshot was taken. At any given time, server is either not
// transmitting (when there are no deltas to transmit), or it is transmitting
// the most recent delta.
//
// Starting the transmission is always preceded with taking a new snapshot,
// calculating the delta relative to the previous snapshot, and clearing the
// state to 'unchanged'. The new snapshot and the delta then remains unchanged
// during transmission.
//
// Upon receiving the change notification, if the server is currently
// transmitting, the state is updated to indicate that the universe has changed,
// but no other action is taken. If the transmission is not in progress, the
// 'snapshot-delta-transmit' process is triggered.
//
// Upon finishing the transfer, if 'change' state is 'changed', the
// 'snapshot-delta-transmit' is immediately triggered again (that is, a new
// delta is calculated and a new transfer is immediately started). Otherwise,
// the 'transmit' state is changed to 'not transmitting'.
//
// All state changes are guarded by a mutex, to synchronize transmits with
// 'change' events.
class UniverseServer : public EventListener {
 public:
  UniverseServer(Universe& universe, UniverseServerChannel& channel,
                 Executor& transmit_executor);

  ~UniverseServer();

  void begin();

  void devicesChanged() override;
  void newReadingsAvailable() override;

 private:
  class State {
   public:
    size_t device_count() const { return devices_.size(); }

    void addDevice(const DeviceLocator& loc,
                   const roo_transceivers_Descriptor& descriptor, int ordinal) {
      newDeviceDelta(loc, DeviceDelta::ADDED, -1);
      int key = addDescriptorReference(descriptor);
      addDeviceEntry(loc, ordinal, key);
    }

    void removeReadings(const DeviceLocator& loc,
                        const roo_transceivers_Descriptor& descriptor) {
      for (size_t i = 0; i < descriptor.sensors_count; ++i) {
        eraseSensorReading(SensorLocator(loc, descriptor.sensors[i].id));
      }
    }

    void removeDevice(const DeviceLocator& loc) {
      const auto& device = devices_[loc];
      int old_descriptor_key = device.descriptor_key;
      int old_ordinal = device.ordinal;
      newDeviceDelta(loc, State::DeviceDelta::REMOVED, old_ordinal);
      const roo_transceivers_Descriptor& old_descriptor =
          descriptors_by_key_[old_descriptor_key];
      removeReadings(loc, old_descriptor);
      removeDescriptorReference(old_descriptor);
      devices_.erase(loc);
    }

    int addDescriptorReference(const roo_transceivers_Descriptor& descriptor);

    void addDeviceEntry(const DeviceLocator& loc, int ordinal,
                        int descriptor_key) {
      devices_[loc] = DeviceEntry{ordinal, descriptor_key};
    }

    void removeDescriptorReference(
        const roo_transceivers_Descriptor& descriptor);

    void eraseSensorReading(const SensorLocator& loc) { readings_.erase(loc); }

    bool updateSensorReading(const SensorLocator& loc, const Measurement& m) {
      auto itr = readings_.find(loc);
      if (itr == readings_.end()) {
        // Did not have prior measurement.
        if (m.isInitial()) return false;
        readings_[loc] = SensorReading{m.value(), m.time()};
      } else {
        // Did have prior measurement.
        auto& v = itr->second;
        if (((m.value() == v.value) ||
             (isnanf(m.value()) && isnanf(v.value))) &&
            (m.time() == v.time)) {
          // The value did not change.
          return false;
        }
        // The value did change. Update, and write to the delta.
        if (m.isInitial()) {
          eraseSensorReading(loc);
        } else {
          v.value = m.value();
          v.time = m.time();
        }
      }
      return true;
    }

    const roo_transceivers_Descriptor& getDescriptor(
        const DeviceLocator& loc) const {
      return descriptors_by_key_[devices_[loc].descriptor_key];
    }

    void clearAll();
    void clearDelta();

    struct DeviceEntry {
      int ordinal;  // the index in the universe.
      int descriptor_key;
    };

    struct DeviceDelta {
      enum Status { ADDED, REMOVED, PRESERVED, MODIFIED };
      DeviceLocator locator;
      Status status;
      int old_ordinal;  // For PRESERVED.
    };

    struct DescriptorDelta {
      enum Status { ADDED, REMOVED };
      int key;
      Status status;
    };

    struct SensorReadingDelta {
      SensorId sensor_id;
      float value;
      roo_time::Uptime time;
    };

    struct SensorReadingDeltaDeviceGroup {
      DeviceLocator device;
      size_t reading_count;
    };

    void newDeviceDelta(const DeviceLocator& loc, DeviceDelta::Status status,
                        int old_ordinal) {
      device_deltas_.emplace_back(DeviceDelta{loc, status, old_ordinal});
    }

    void newDescriptorDelta(int key, DescriptorDelta::Status kind) {
      descriptor_deltas_.emplace_back(DescriptorDelta{key, kind});
    }

    void newSensorReadingDelta(const SensorLocator& loc, float value,
                               roo_time::Uptime time);

    const std::vector<DeviceDelta>& device_deltas() const {
      return device_deltas_;
    }

    const std::vector<DescriptorDelta>& descriptor_deltas() const {
      return descriptor_deltas_;
    }

    const roo_collections::FlatSmallHashMap<DeviceLocator, DeviceEntry>&
    devices() const {
      return devices_;
    }

    const roo_collections::FlatSmallHashMap<int, roo_transceivers_Descriptor>&
    descriptors_by_key() const {
      return descriptors_by_key_;
    }

    const std::vector<SensorReadingDeltaDeviceGroup>& reading_delta_groups()
        const {
      return reading_delta_groups_;
    }

    const std::vector<SensorReadingDelta>& reading_deltas() const {
      return reading_deltas_;
    }

   private:
    struct SensorReading {
      float value;
      roo_time::Uptime time;
    };

    roo_collections::FlatSmallHashMap<DeviceLocator, DeviceEntry> devices_;

    DescriptorMap descriptors_;

    roo_collections::FlatSmallHashMap<int, roo_transceivers_Descriptor>
        descriptors_by_key_;

    roo_collections::FlatSmallHashMap<SensorLocator, SensorReading> readings_;

    std::vector<DeviceDelta> device_deltas_;
    std::vector<DescriptorDelta> descriptor_deltas_;

    // Devices appear in the order of enumeration by the universe.
    std::vector<SensorReadingDeltaDeviceGroup> reading_delta_groups_;
    std::vector<SensorReadingDelta> reading_deltas_;
  };

  void handleClientMessage(const roo_transceivers_ClientMessage& msg);

  void handleRequestState();

  void triggerTransmission();

  void snapshotDevices();
  void snapshotSensorState();

  // Send the handhake message.
  void transmitInit();

  void transmissionLoop();

  // Sends a single delta or snapshot over the channel.
  void transmit(bool is_delta);

  void transmitUpdateBegin(bool delta);
  void transmitUpdateEnd();

  void transmitDescriptorAdded(int key);
  void transmitDescriptorRemoved(int key);

  void transmitDeviceAdded(const DeviceLocator& locator, int descriptor_key);
  void transmitDevicesPreserved(int first_preserved_ordinal, size_t count);
  void transmitDeviceModified(int prev_ordinal, int descriptor_key);
  void transmitDeviceRemoved(int prev_ordinal);

  void transmitReadingsBegin();
  void transmitReadingsEnd();

  Universe& universe_;
  UniverseServerChannel& channel_;
  Executor& transmit_executor_;

  State state_;

  bool full_snapshot_requested_;
  bool is_full_snapshot_;
  bool transmission_in_progress_;
  bool device_update_pending_;
  bool readings_pending_;

  mutable roo::mutex state_guard_;
};

}  // namespace roo_transceivers