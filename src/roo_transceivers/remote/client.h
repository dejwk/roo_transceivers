#pragma once

#include <functional>
#include <mutex>
#include <vector>

#include "roo_collections/flat_small_hash_map.h"
#include "roo_collections/flat_small_hash_set.h"
#include "roo_transceivers/id.h"
#include "roo_transceivers/notification.h"
#include "roo_transceivers/universe.h"

namespace roo_transceivers {

class UniverseClientChannel {
 public:
  using ServerMessageCb =
      std::function<void(const roo_transceivers_ServerMessage&)>;

  virtual ~UniverseClientChannel() = default;

  virtual void registerServerMessageCallback(ServerMessageCb cb) = 0;

  virtual void sendClientMessage(const roo_transceivers_ClientMessage& msg) = 0;
};

// The universe that mirrors a remote universe, with with it synchronizes the
// state using a bi-directional serial communication channel.
class UniverseClient : public Universe {
 public:
  struct DeviceEntry {
    DeviceLocator locator;
    int descriptor_key;
  };

  UniverseClient(UniverseClientChannel& channel);

  ~UniverseClient();

  size_t deviceCount() const override;

  bool forEachDevice(
      std::function<bool(const DeviceLocator&)> callback) const override;

  bool getDeviceDescriptor(
      const DeviceLocator& locator,
      roo_transceivers_Descriptor& descriptor) const override;

  Measurement read(const SensorLocator& locator) const override;

  bool write(const ActuatorLocator& locator, float value) const override;

  void requestUpdate() override;

  void addEventListener(EventListener* listener) override;

  void removeEventListener(EventListener* listener) override;

 private:
  // Convenience function to lookup a descriptor by device locator. Additionally
  // returns the descriptor key, assigned by the server.
  const roo_transceivers_Descriptor* lookupDeviceDescriptor(
      const DeviceLocator& locator, int& descriptor_key) const;

  // Called when a message is received from the server.
  void handleServerMessage(const roo_transceivers_ServerMessage& msg);

  void handleDescriptorAdded(int key,
                             const roo_transceivers_Descriptor& descriptor);

  void handleDescriptorRemoved(int key);

  void handleDevice(const DeviceLocator& locator, int descriptor_key);

  void handleDeviceAdded(const DeviceLocator& locator, int descriptor_key);

  void handleDeviceRemoved(int prev_index);

  void handleDevicePreserved(int prev_index_first, size_t count);

  void handleDeviceModified(int prev_index, int descriptor_key);

  // Helper method to propagate the devices event to listeners.
  void notifyDevicesChanged();

  // Helper method to propagate the readings event to listeners.
  void notifyReadingsAvailable();

  void clearAll();

  void handleUpdateEnd();

  void handleReadingsBegin();

  void handleReadings(
      const DeviceLocator& device,
      const roo_transceivers_ServerMessage_Reading_SensorValue* readings,
      size_t readings_count);

  void handleReadingsEnd();

  // The channel used to synchronize the state (i.e., receive measurements from
  // and push writes to) with the remote universe.
  UniverseClientChannel& channel_;

  // Stores all descriptors for devices handled by the remote universe. The
  // descriptors are assigned integer keys by the remote universe. These are
  // used as keys into this map.
  roo_collections::FlatSmallHashMap<int, roo_transceivers_Descriptor>
      descriptors_;

  // For each device known to the remote universe, points to the key of
  // the descriptor for that device. That key can be used to look up the
  // descriptor in the descriptors_ map.
  roo_collections::FlatSmallHashMap<DeviceLocator, int> device_idx_by_locator_;

  std::vector<DeviceEntry> devices_;

  std::vector<DeviceEntry> updated_devices_;

  // Cached sensor measurements.
  roo_collections::FlatSmallHashMap<SensorLocator, Measurement> readings_;

  // Actuators by actuator ID per device type. Allows us to quickly verify that
  // an actuator locator used for write actually does exist, before sending the
  // write request to the server.
  roo_collections::FlatSmallHashSet<ActuatorLocator> actuators_;

  // Event listeners for state updates.
  roo_collections::FlatSmallHashSet<EventListener*> listeners_;

  mutable std::mutex state_guard_;
  mutable std::mutex listener_guard_;
};

}  // namespace roo_transceivers