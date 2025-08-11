#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "roo_transceivers.h"
#include "roo_transceivers/helpers/transceiver_collection.h"
#include "roo_transceivers/remote/client.h"
#include "roo_transceivers/remote/proto.h"
#include "roo_transceivers/remote/server.h"

void PrintTo(const roo_transceivers_ServerMessage& msg, std::ostream* os) {
  (*os) << "\n{\n";
  switch (msg.which_contents) {
    case roo_transceivers_ServerMessage_init_tag: {
      (*os) << "  Init{}\n";
      break;
    }
    case roo_transceivers_ServerMessage_transceiver_update_begin_tag: {
      (*os) << "  UpdateBegin { delta = "
            << msg.contents.transceiver_update_begin.delta << " }\n";
      break;
    }
    case roo_transceivers_ServerMessage_descriptor_added_tag: {
      (*os) << "  DescriptorAdded { key = " << msg.contents.descriptor_added.key
            << " }\n";
      break;
    }
    case roo_transceivers_ServerMessage_descriptor_removed_tag: {
      (*os) << "  DescriptorAdded { key = "
            << msg.contents.descriptor_removed.key << " }\n";
      break;
    }
    case roo_transceivers_ServerMessage_device_added_tag: {
      (*os) << "  DeviceAdded { locator = "
            << roo_transceivers::DeviceLocator(
                   msg.contents.device_added.locator_schema,
                   msg.contents.device_added.locator_id)
                   .toString()
            << ", descriptor_key: " << msg.contents.device_added.descriptor_key
            << " }\n";
      break;
    }
    case roo_transceivers_ServerMessage_device_removed_tag: {
      (*os) << "  DeviceRemoved { prev_index = "
            << msg.contents.device_removed.prev_index << " }\n";
      break;
    }
    case roo_transceivers_ServerMessage_device_preserved_tag: {
      (*os) << "  DevicePreserved { prev_index = "
            << msg.contents.device_preserved.prev_index
            << ", count = " << msg.contents.device_preserved.count << " }\n";
      break;
    }
    case roo_transceivers_ServerMessage_device_modified_tag: {
      (*os) << "  DeviceModified { prev_index = "
            << msg.contents.device_modified.prev_index << ", descriptor_key = "
            << msg.contents.device_modified.descriptor_key << " }\n";
      break;
    }
    case roo_transceivers_ServerMessage_transceiver_update_end_tag: {
      (*os) << "  UpdateEnd{}\n";
      break;
    }
    case roo_transceivers_ServerMessage_readings_begin_tag: {
      (*os) << "  ReadingsBegin{}\n";
      break;
    }
    case roo_transceivers_ServerMessage_readings_end_tag: {
      (*os) << "  ReadingsEnd{}\n";
      break;
    }
    case roo_transceivers_ServerMessage_reading_tag: {
      (*os) << "  Reading { device = "
            << roo_transceivers::DeviceLocator(
                   msg.contents.reading.device_locator_schema,
                   msg.contents.reading.device_locator_id)
                   .toString()
            << ", sensors: [";
      for (size_t i = 0; i < msg.contents.reading.sensor_values_count; ++i) {
        auto& reading = msg.contents.reading.sensor_values[i];
        if (i > 0) {
          (*os) << ", ";
        }
        (*os) << "(\"" << reading.device_locator_sensor_id << "\" : "
              << reading.value << ")";
      }
      (*os) << " }\n";
      break;
    }
    default: {
      (*os) << "  (Unknown case)\n";
      break;
    }
  }
  (*os) << "}\n";
}
namespace roo_transceivers {

using testing::_;
using testing::InSequence;
using testing::Sequence;

class FakeThermometer : public SimpleSensor {
 public:
  FakeThermometer()
      : SimpleSensor(roo_transceivers_Quantity_kTemperature),
        temperature_deg_c_(std::nan("")) {}

  void set(float temp_c) { temperature_deg_c_ = temp_c; }

 protected:
  float readFromSensor() const override { return temperature_deg_c_; }

 private:
  float temperature_deg_c_;
};

class MockChannel : public UniverseServerChannel {
 public:
  MockChannel() = default;

  MOCK_METHOD1(registerClientMessageCallback, void(ClientMessageCb cb));

  MOCK_METHOD1(sendServerMessage,
               void(const roo_transceivers_ServerMessage& msg));
};

class DirectExecutor : public Executor {
 public:
  void execute(std::function<void()> task) override { task(); }
};

MATCHER_P(MsgEq, msg, "") {
  if (arg.which_contents != msg.which_contents) return false;
  switch (arg.which_contents) {
    case roo_transceivers_ServerMessage_transceiver_update_begin_tag: {
      return arg.contents.transceiver_update_begin.delta ==
             msg.contents.transceiver_update_begin.delta;
    }
    case roo_transceivers_ServerMessage_descriptor_added_tag: {
      const auto& a = arg.contents.descriptor_added;
      const auto& b = msg.contents.descriptor_added;
      return a.key == b.key && a.descriptor == b.descriptor;
    }
    case roo_transceivers_ServerMessage_descriptor_removed_tag: {
      const auto& a = arg.contents.descriptor_removed;
      const auto& b = msg.contents.descriptor_removed;
      return a.key == b.key;
    }
    case roo_transceivers_ServerMessage_device_added_tag: {
      const auto& a = arg.contents.device_added;
      const auto& b = msg.contents.device_added;
      return strncmp(a.locator_schema, b.locator_schema,
                     DeviceSchema::kCapacity) == 0 &&
             strncmp(a.locator_id, b.locator_id, DeviceId::kCapacity) == 0 &&
             a.descriptor_key == b.descriptor_key;
    }
    case roo_transceivers_ServerMessage_device_removed_tag: {
      const auto& a = arg.contents.device_removed;
      const auto& b = msg.contents.device_removed;
      return a.prev_index == b.prev_index;
    }
    case roo_transceivers_ServerMessage_device_preserved_tag: {
      const auto& a = arg.contents.device_preserved;
      const auto& b = msg.contents.device_preserved;
      return a.prev_index == b.prev_index && a.count == b.count;
    }
    case roo_transceivers_ServerMessage_device_modified_tag: {
      const auto& a = arg.contents.device_modified;
      const auto& b = msg.contents.device_modified;
      return a.prev_index == b.prev_index &&
             a.descriptor_key == b.descriptor_key;
    }
    case roo_transceivers_ServerMessage_reading_tag: {
      const auto& a = arg.contents.reading;
      const auto& b = msg.contents.reading;
      if (strncmp(a.device_locator_schema, b.device_locator_schema,
                  DeviceSchema::kCapacity) != 0) {
        return false;
      }
      if (strncmp(a.device_locator_id, b.device_locator_id,
                  DeviceId::kCapacity) != 0) {
        return false;
      }
      if (a.sensor_values_count != b.sensor_values_count) return false;
      for (size_t i = 0; i < a.sensor_values_count; ++i) {
        if (strncmp(a.sensor_values[i].device_locator_sensor_id,
                    a.sensor_values[i].device_locator_sensor_id,
                    SensorId::kCapacity) != 0) {
          return false;
        }
        if (!(isnan(a.sensor_values[i].value) &&
              isnan(b.sensor_values[i].value)) &&
            a.sensor_values[i].value != b.sensor_values[i].value) {
          return false;
        }
      }
      return true;
    }
    default: {
      return true;
    }
  }
}

// class ServerTest : public testing::Test {
//  protected:
//   MockChannel channel_;
// };

TEST(ServerTest, SendInit) {
  FakeThermometer t1;
  TransceiverCollection universe({{DeviceLocator("temp", "t1"), &t1}});
  DirectExecutor executor;
  MockChannel channel;
  {
    InSequence s;
    EXPECT_CALL(channel, registerClientMessageCallback(_));
    EXPECT_CALL(channel, sendServerMessage(MsgEq(proto::SrvInit())));
    EXPECT_CALL(channel, registerClientMessageCallback(_));
  }
  UniverseServer server(universe, channel, executor);
  server.begin();
}

TEST(ServerTest, SendInitAndDevicesUpdated) {
  FakeThermometer t1;
  DeviceLocator loc("temp", "t1");
  TransceiverCollection universe({{loc, &t1}});
  DirectExecutor executor;
  MockChannel channel;
  roo_transceivers_Descriptor descriptor;
  t1.getDescriptor(descriptor);
  {
    InSequence s;
    EXPECT_CALL(channel, registerClientMessageCallback(_));
    EXPECT_CALL(channel, sendServerMessage(MsgEq(proto::SrvInit())));
    EXPECT_CALL(channel, sendServerMessage(MsgEq(proto::SrvFullUpdateBegin())));
    EXPECT_CALL(channel, sendServerMessage(
                             MsgEq(proto::SrvDescriptorAdded(0, descriptor))));
    EXPECT_CALL(channel,
                sendServerMessage(MsgEq(proto::SrvDeviceAdded(loc, 0))));
    EXPECT_CALL(channel, sendServerMessage(MsgEq(proto::SrvUpdateEnd())));
    EXPECT_CALL(channel, sendServerMessage(MsgEq(proto::SrvReadingsBegin())));
    auto reading = proto::SrvReading(loc);
    proto::AddReading(reading, SensorId(""), nanf(""), 0);
    EXPECT_CALL(channel, sendServerMessage(MsgEq(reading)));
    EXPECT_CALL(channel, sendServerMessage(MsgEq(proto::SrvReadingsEnd())));
    EXPECT_CALL(channel, registerClientMessageCallback(_));
  }
  UniverseServer server(universe, channel, executor);
  server.begin();
  server.devicesChanged();
}

}  // namespace roo_transceivers