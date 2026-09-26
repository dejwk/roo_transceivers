#pragma once

#include "roo_transceivers.pb.h"
#include "roo_transceivers/id.h"

namespace roo_transceivers {
namespace proto {

/// Builds an init server message.
roo_transceivers::ServerMessage SrvInit();

/// Begins a full update sequence.
roo_transceivers::ServerMessage SrvFullUpdateBegin();

/// Begins a delta update sequence.
roo_transceivers::ServerMessage SrvDeltaUpdateBegin();

/// Ends an update sequence.
roo_transceivers::ServerMessage SrvUpdateEnd();

/// Adds a descriptor in the server stream.
roo_transceivers::ServerMessage SrvDescriptorAdded(
    int key, const roo_transceivers::Descriptor& descriptor);

/// Removes a descriptor from the server stream.
roo_transceivers::ServerMessage SrvDescriptorRemoved(int key);

/// Adds a device with a descriptor key.
roo_transceivers::ServerMessage SrvDeviceAdded(const DeviceLocator& locator,
                                               int descriptor_key);

/// Marks a range of devices as preserved.
roo_transceivers::ServerMessage SrvDevicesPreserved(int first_preserved_ordinal,
                                                    size_t count);

/// Marks a device as modified.
roo_transceivers::ServerMessage SrvDevicesModified(int prev_ordinal,
                                                   int descriptor_key);

/// Removes a device.
roo_transceivers::ServerMessage SrvDeviceRemoved(int prev_ordinal);

/// Begins a readings block.
roo_transceivers::ServerMessage SrvReadingsBegin();

/// Ends a readings block.
roo_transceivers::ServerMessage SrvReadingsEnd();

/// Begins readings for a device.
roo_transceivers::ServerMessage SrvReading(const DeviceLocator& device);

/// Appends a single sensor reading to a readings message.
void AddReading(roo_transceivers::ServerMessage& reading,
                const SensorId& sensor_id, float value, uint64_t age_ms);

/// Builds a client update request.
roo_transceivers::ClientMessage ClientRequestUpdate();
/// Builds a client state request.
roo_transceivers::ClientMessage ClientRequestState();
/// Builds a client write request.
roo_transceivers::ClientMessage ClientWrite(const ActuatorLocator& actuator,
                                            float value);

}  // namespace proto

}  // namespace roo_transceivers