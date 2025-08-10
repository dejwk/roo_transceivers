#include "roo_transceivers/helpers/transceiver_collection.h"

#include <unordered_set>

#include "gtest/gtest.h"

namespace roo_transceivers {

// Dummy implementations for required types.
struct DummyDescriptor : public roo_transceivers_Descriptor {
  DummyDescriptor() {
    sensors_count = 0;
    actuators_count = 0;
  }
};

class DummyTransceiver : public Transceiver {
 public:
  DummyTransceiver(int id) {}

  void getDescriptor(roo_transceivers_Descriptor& descriptor) const override {
    descriptor = DummyDescriptor();
  }
  Measurement read(const SensorId&) const override { return Measurement(); }
  bool write(const ActuatorId&, float) override { return false; }
  void addEventListener(EventListener* listener) override {
    listeners_.insert(listener);
  }
  void removeEventListener(EventListener* listener) override {
    listeners_.erase(listener);
  }
  void requestUpdate() override { ++update_count_; }
  int update_count() const { return update_count_; }

 private:
  int update_count_ = 0;
  roo_collections::FlatSmallHashSet<EventListener*> listeners_;
};

class DummyListener : public EventListener {
 public:
  void devicesChanged() override { ++devices_changed_count_; }
  void newReadingsAvailable() override { ++new_readings_count_; }
  int devices_changed_count() const { return devices_changed_count_; }
  int new_readings_count() const { return new_readings_count_; }

 private:
  int devices_changed_count_ = 0;
  int new_readings_count_ = 0;
};

TEST(TransceiverCollectionTest, AddAndCountTransceivers) {
  TransceiverCollection collection;
  DummyTransceiver t1(1);
  DummyTransceiver t2(2);
  DeviceLocator l1("schema", "dev1");
  DeviceLocator l2("schema", "dev2");
  EXPECT_EQ(collection.deviceCount(), 0);
  collection.add(l1, &t1);
  EXPECT_EQ(collection.deviceCount(), 1);
  collection.add(l2, &t2);
  EXPECT_EQ(collection.deviceCount(), 2);
}

TEST(TransceiverCollectionTest, ForEachDevice) {
  TransceiverCollection collection;
  DummyTransceiver t1(1);
  DummyTransceiver t2(2);
  DeviceLocator l1("schema", "dev1");
  DeviceLocator l2("schema", "dev2");
  collection.add(l1, &t1);
  collection.add(l2, &t2);
  std::unordered_set<DeviceLocator> found;
  collection.forEachDevice([&](const DeviceLocator& loc) {
    found.insert(loc);
    return true;
  });
  EXPECT_EQ(found.size(), 2);
  EXPECT_GT(found.count(l1), 0);
  EXPECT_GT(found.count(l2), 0);
}

TEST(TransceiverCollectionTest, GetDeviceDescriptor) {
  TransceiverCollection collection;
  DummyTransceiver t1(1);
  DeviceLocator l1("schema", "dev1");
  collection.add(l1, &t1);
  roo_transceivers_Descriptor desc;
  ASSERT_TRUE(collection.getDeviceDescriptor(l1, desc));
  DeviceLocator l2("schema", "dev2");
  EXPECT_FALSE(collection.getDeviceDescriptor(l2, desc));
}

TEST(TransceiverCollectionTest, ReadAndWriteDelegation) {
  class RWTransceiver : public DummyTransceiver {
   public:
    RWTransceiver() : DummyTransceiver(0) {}
    Measurement read(const SensorId& id) const override {
      if (id == "sensor") {
        return Measurement(roo_transceivers_Quantity_kTemperature,
                           roo_time::Uptime::Now(), 42.0f);
      }
      return Measurement();
    }
    bool write(const ActuatorId& id, float value) override {
      if (id == "act" && value == 123.0f) return true;
      return false;
    }
    void getDescriptor(roo_transceivers_Descriptor& descriptor) const override {
      descriptor = DummyDescriptor();
    }
    void addEventListener(EventListener*) override {}
    void removeEventListener(EventListener*) override {}
  };
  TransceiverCollection collection;
  RWTransceiver t;
  DeviceLocator l("schema", "dev");
  collection.add(l, &t);

  SensorLocator s_loc(l, "sensor");
  Measurement m = collection.read(s_loc);
  EXPECT_EQ(m.value(), 42.0f);

  ActuatorLocator a_loc(l, "act");
  EXPECT_TRUE(collection.write(a_loc, 123.0f));
  EXPECT_FALSE(collection.write(a_loc, 999.0f));
}

TEST(TransceiverCollectionTest, RequestUpdatePropagates) {
  TransceiverCollection collection;
  DummyTransceiver t1(1);
  DummyTransceiver t2(2);
  DeviceLocator l1("schema", "dev1");
  DeviceLocator l2("schema", "dev2");
  collection.add(l1, &t1);
  collection.add(l2, &t2);
  collection.requestUpdate();
  EXPECT_EQ(t1.update_count(), 1);
  EXPECT_EQ(t2.update_count(), 1);
}

TEST(TransceiverCollectionTest, EventListenerNotification) {
  TransceiverCollection collection;
  DummyListener listener1, listener2;
  collection.addEventListener(&listener1);
  collection.addEventListener(&listener2);

  collection.devicesChanged();
  EXPECT_EQ(listener1.devices_changed_count(), 1);
  EXPECT_EQ(listener2.devices_changed_count(), 1);

  collection.newReadingsAvailable();
  EXPECT_EQ(listener1.new_readings_count(), 1);
  EXPECT_EQ(listener2.new_readings_count(), 1);

  collection.removeEventListener(&listener2);
  collection.devicesChanged();
  EXPECT_EQ(listener1.devices_changed_count(), 2);
  EXPECT_EQ(listener2.devices_changed_count(), 1);
}

TEST(TransceiverCollectionTest, AddDuplicateLocatorThrows) {
  TransceiverCollection collection;
  DummyTransceiver t1(1);
  DummyTransceiver t2(2);
  DeviceLocator l1("schema", "dev1");
  collection.add(l1, &t1);
#ifdef NDEBUG
  // In release mode, CHECK may not abort, so just skip.
  SUCCEED();
#else
  // In debug mode, CHECK should abort, so expect death.
  EXPECT_DEATH(collection.add(l1, &t2), "Duplicate device locator");
#endif
}

TEST(TransceiverCollectionTest, ConstructorWithVector) {
  DummyTransceiver t1(1);
  DummyTransceiver t2(2);
  DeviceLocator l1("schema", "dev1");
  DeviceLocator l2("schema", "dev2");
  std::vector<TransceiverCollection::Entry> entries = {{l1, &t1}, {l2, &t2}};
  TransceiverCollection collection(entries);
  EXPECT_EQ(collection.deviceCount(), 2);
  roo_transceivers_Descriptor desc;
  EXPECT_TRUE(collection.getDeviceDescriptor(l1, desc));
  EXPECT_TRUE(collection.getDeviceDescriptor(l2, desc));
}

}  // namespace roo_transceivers