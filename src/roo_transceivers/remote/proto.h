#pragma once

#include "roo_transceivers.pb.h"
#include "roo_transceivers/id.h"

namespace roo_transceivers {
namespace proto {

roo_transceivers_ServerMessage SrvInit();

roo_transceivers_ServerMessage SrvFullUpdateBegin();

roo_transceivers_ServerMessage SrvDeltaUpdateBegin();

roo_transceivers_ServerMessage SrvUpdateEnd();

roo_transceivers_ServerMessage SrvDescriptorAdded(
    int key, const roo_transceivers_Descriptor& descriptor);

roo_transceivers_ServerMessage SrvDescriptorRemoved(int key);

roo_transceivers_ServerMessage SrvDeviceAdded(const DeviceLocator& locator,
                                              int descriptor_key);

roo_transceivers_ServerMessage SrvDevicesPreserved(int first_preserved_ordinal,
                                                   size_t count);

roo_transceivers_ServerMessage SrvDevicesModified(int prev_ordinal,
                                                  int descriptor_key);

roo_transceivers_ServerMessage SrvDevicesRemoved(int prev_ordinal);

roo_transceivers_ServerMessage SrvReadingsBegin();

roo_transceivers_ServerMessage SrvReadingsEnd();

roo_transceivers_ServerMessage SrvReading(const DeviceLocator& device);

void AddReading(roo_transceivers_ServerMessage& reading,
                const SensorId& sensor_id, float value, uint64_t age_ms);

}  // namespace proto

}  // namespace roo_transceivers