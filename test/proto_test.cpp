#include "roo_transceivers/remote/proto.h"

#include <initializer_list>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "roo_transceivers/descriptor.h"
#include "roo_transceivers/remote/server.h"

namespace roo_transceivers {
namespace {

template <typename Message>
void ExpectWire(const Message& message,
                std::initializer_list<uint8_t> expected) {
  uint8_t buffer[Message::kMaxEncodedSize];
  size_t size = 0;
  ASSERT_EQ(roo_pb::Serialize(message, buffer, sizeof(buffer), size),
            roo_pb::Status::kOk);
  EXPECT_EQ(std::vector<uint8_t>(buffer, buffer + size),
            std::vector<uint8_t>(expected));
  Message parsed;
  ASSERT_TRUE(parsed.ParseFromArray(expected.begin(), expected.size()));
  size_t parsed_size = 0;
  ASSERT_EQ(roo_pb::Serialize(parsed, buffer, sizeof(buffer), parsed_size),
            roo_pb::Status::kOk);
  EXPECT_EQ(std::vector<uint8_t>(buffer, buffer + parsed_size),
            std::vector<uint8_t>(expected));
}

// Verifies existing field numbers, empty oneof presence, and optional counts.
TEST(ProtoTest, ControlMessageWireFormat) {
  ExpectWire(proto::ClientRequestUpdate(), {0x0a, 0x00});
  ExpectWire(proto::ClientRequestState(), {0x12, 0x00});
  ExpectWire(proto::SrvInit(), {0x0a, 0x00});
  ExpectWire(proto::SrvFullUpdateBegin(), {0x12, 0x00});
  ExpectWire(proto::SrvDeltaUpdateBegin(), {0x12, 0x02, 0x08, 0x01});
  ExpectWire(proto::SrvUpdateEnd(), {0x4a, 0x00});
  ExpectWire(proto::SrvReadingsBegin(), {0x52, 0x00});
  ExpectWire(proto::SrvReadingsEnd(), {0x62, 0x00});
  ExpectWire(proto::SrvDescriptorRemoved(7), {0x22, 0x02, 0x08, 0x07});
  ExpectWire(proto::SrvDeviceRemoved(7), {0x32, 0x02, 0x08, 0x07});
  ExpectWire(proto::SrvDevicesPreserved(7, 1), {0x3a, 0x02, 0x08, 0x07});
  ExpectWire(proto::SrvDevicesPreserved(7, 2),
             {0x3a, 0x04, 0x08, 0x07, 0x10, 0x02});
  ExpectWire(proto::SrvDevicesModified(7, 2),
             {0x42, 0x04, 0x08, 0x07, 0x18, 0x02});
  auto explicit_zero = proto::SrvDevicesPreserved(7, 1);
  explicit_zero.mutable_device_preserved()->set_count(0);
  ExpectWire(explicit_zero, {0x3a, 0x04, 0x08, 0x07, 0x10, 0x00});
}

// Verifies nested descriptors, identifiers, floats, and reading ages on the
// wire.
TEST(ProtoTest, PayloadWireFormat) {
  Descriptor descriptor;
  auto* sensor = descriptor.add_sensors();
  sensor->set_id("t");
  sensor->set_quantity(Quantity::kTemperature);
  ExpectWire(proto::SrvDescriptorAdded(7, descriptor),
             {0x1a, 0x0b, 0x08, 0x07, 0x12, 0x07, 0x1a, 0x05, 0x0a, 0x01, 't',
              0x10, 0x03});
  ExpectWire(proto::SrvDeviceAdded(DeviceLocator("s", "d"), 7),
             {0x2a, 0x08, 0x0a, 0x01, 's', 0x12, 0x01, 'd', 0x18, 0x07});
  ExpectWire(proto::ClientWrite(ActuatorLocator("s", "d", "a"), 1.0f),
             {0x1a, 0x0e, 0x0a, 0x01, 's', 0x12, 0x01, 'd', 0x1a, 0x01, 'a',
              0x25, 0x00, 0x00, 0x80, 0x3f});
  auto reading = proto::SrvReading(DeviceLocator("s", "d"));
  proto::AddReading(reading, SensorId("t"), 1.0f, 150);
  ExpectWire(reading,
             {0x5a, 0x13, 0x0a, 0x01, 's',  0x12, 0x01, 'd',  0x1a, 0x0b, 0x0a,
              0x01, 't',  0x15, 0x00, 0x00, 0x80, 0x3f, 0x18, 0x96, 0x01});
}

// Verifies the old string/array bounds and copying of a full oneof payload.
TEST(ProtoTest, BoundedMessagesRoundTrip) {
  const std::string schema(15, 's');
  const std::string id(23, 'd');
  auto reading = proto::SrvReading(DeviceLocator(schema.c_str(), id.c_str()));
  for (int i = 0; i < 16; ++i) {
    proto::AddReading(reading, SensorId(id.c_str()), static_cast<float>(i), i);
  }
  EXPECT_EQ(reading.mutable_reading()->try_add_sensor_values(), nullptr);
  EXPECT_FALSE(reading.mutable_reading()->try_set_device_locator_schema(
      std::string(16, 'x').c_str()));
  EXPECT_FALSE(reading.mutable_reading()->try_set_device_locator_id(
      std::string(24, 'x').c_str()));
  ServerMessage copy = reading;
  reading = proto::SrvInit();
  uint8_t buffer[ServerMessage::kMaxEncodedSize];
  size_t size = 0;
  ASSERT_EQ(roo_pb::Serialize(copy, buffer, sizeof(buffer), size),
            roo_pb::Status::kOk);
  ServerMessage parsed;
  ASSERT_TRUE(parsed.ParseFromArray(buffer, size));
  ASSERT_EQ(parsed.reading().sensor_values_size(), 16u);
  EXPECT_STREQ(parsed.reading().device_locator_schema().c_str(),
               schema.c_str());
  EXPECT_STREQ(parsed.reading().device_locator_id().c_str(), id.c_str());
  for (int i = 0; i < 16; ++i) {
    EXPECT_STREQ(
        parsed.reading().sensor_values(i).device_locator_sensor_id().c_str(),
        id.c_str());
    EXPECT_EQ(parsed.reading().sensor_values(i).value(), i);
    EXPECT_EQ(parsed.reading().sensor_values(i).age_ms(),
              static_cast<uint64_t>(i));
  }
  Descriptor descriptor;
  for (int i = 0; i < 16; ++i) {
    descriptor.add_sensors()->set_id(id.c_str());
    descriptor.add_actuators()->set_id(id.c_str());
  }
  EXPECT_EQ(descriptor.try_add_sensors(), nullptr);
  EXPECT_EQ(descriptor.try_add_actuators(), nullptr);
  EXPECT_FALSE(
      descriptor.mutable_sensors(0)->try_set_id(std::string(24, 'x').c_str()));
  auto added = proto::SrvDescriptorAdded(7, descriptor);
  ASSERT_EQ(roo_pb::Serialize(added, buffer, sizeof(buffer), size),
            roo_pb::Status::kOk);
  ASSERT_TRUE(parsed.ParseFromArray(buffer, size));
  EXPECT_TRUE(parsed.descriptor_added().has_descriptor());
  EXPECT_TRUE(parsed.descriptor_added().descriptor() == descriptor);
  EXPECT_EQ(DescriptorHashFn()(parsed.descriptor_added().descriptor()),
            DescriptorHashFn()(descriptor));
}

}  // namespace
}  // namespace roo_transceivers
