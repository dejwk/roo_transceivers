#include "gtest/gtest.h"
#include "roo_transceivers.h"
#include "roo_transceivers/id.h"

namespace roo_transceivers {

// DeviceLocator tests.

TEST(DeviceLocatorTest, DefaultConstructor) {
  DeviceLocator loc;
  EXPECT_TRUE(loc.schema().empty());
  EXPECT_TRUE(loc.device_id().empty());
  EXPECT_FALSE(loc.isDefined());
}

TEST(DeviceLocatorTest, ParameterizedConstructor) {
  DeviceLocator loc("myschema", "dev123");
  EXPECT_EQ(loc.schema(), "myschema");
  EXPECT_EQ(loc.device_id(), "dev123");
  EXPECT_TRUE(loc.isDefined());
}

TEST(DeviceLocatorTest, EqualityOperators) {
  DeviceLocator a("myschema", "dev123");
  DeviceLocator b("myschema", "dev123");
  DeviceLocator c("otherschema", "dev123");
  DeviceLocator d("myschema", "otherdev");
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_FALSE(a == c);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == d);
  EXPECT_TRUE(a != d);
}

TEST(DeviceLocatorTest, ToStringAndWriteCstr) {
  DeviceLocator loc("myschema", "dev123");
  std::string s = loc.toString();
  EXPECT_FALSE(s.empty());
  char buf[64];
  loc.write_cstr(buf);
  EXPECT_EQ(buf, s);
  EXPECT_EQ("myschema:dev123", s);
}

// SensorLocator tests.

TEST(SensorLocatorTest, DefaultConstructor) {
  SensorLocator loc;
  EXPECT_TRUE(loc.schema().empty());
  EXPECT_TRUE(loc.device_id().empty());
  EXPECT_TRUE(loc.sensor_id().empty());
  EXPECT_FALSE(loc.isDefined());
}

TEST(SensorLocatorTest, ParameterizedConstructor) {
  SensorLocator loc("myschema", "dev123", "sensorA");
  EXPECT_EQ(loc.schema(), "myschema");
  EXPECT_EQ(loc.device_id(), "dev123");
  EXPECT_EQ(loc.sensor_id(), "sensorA");
  EXPECT_TRUE(loc.isDefined());
}

TEST(SensorLocatorTest, DeviceLocatorConstructor) {
  DeviceLocator dev("myschema", "dev123");
  SensorLocator loc(dev, "sensorA");
  EXPECT_EQ(loc.schema(), "myschema");
  EXPECT_EQ(loc.device_id(), "dev123");
  EXPECT_EQ(loc.sensor_id(), "sensorA");
}

TEST(SensorLocatorTest, EqualityOperators) {
  SensorLocator a("myschema", "dev123", "sensorA");
  SensorLocator b("myschema", "dev123", "sensorA");
  SensorLocator c("myschema", "dev123", "sensorB");
  SensorLocator d("myschema", "dev456", "sensorA");
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_FALSE(a == c);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == d);
  EXPECT_TRUE(a != d);
}

TEST(SensorLocatorTest, ToStringAndWriteCstr) {
  SensorLocator loc("myschema", "dev123", "sensorA");
  std::string s = loc.toString();
  EXPECT_FALSE(s.empty());
  char buf[64];
  loc.write_cstr(buf);
  EXPECT_EQ(buf, s);
  EXPECT_EQ(s, "myschema:dev123/sensorA");
}

// ActuatorLocator tests.

TEST(ActuatorLocatorTest, DefaultConstructor) {
  ActuatorLocator loc;
  EXPECT_TRUE(loc.schema().empty());
  EXPECT_TRUE(loc.device_id().empty());
  EXPECT_TRUE(loc.actuator_id().empty());
  EXPECT_FALSE(loc.isDefined());
}

TEST(ActuatorLocatorTest, ParameterizedConstructor) {
  ActuatorLocator loc("myschema", "dev123", "actuatorA");
  EXPECT_EQ(loc.schema(), "myschema");
  EXPECT_EQ(loc.device_id(), "dev123");
  EXPECT_EQ(loc.actuator_id(), "actuatorA");
  EXPECT_TRUE(loc.isDefined());
}

TEST(ActuatorLocatorTest, DeviceLocatorConstructor) {
  DeviceLocator dev("myschema", "dev123");
  ActuatorLocator loc(dev, "actuatorA");
  EXPECT_EQ(loc.schema(), "myschema");
  EXPECT_EQ(loc.device_id(), "dev123");
  EXPECT_EQ(loc.actuator_id(), "actuatorA");
}

TEST(ActuatorLocatorTest, EqualityOperators) {
  ActuatorLocator a("myschema", "dev123", "actuatorA");
  ActuatorLocator b("myschema", "dev123", "actuatorA");
  ActuatorLocator c("myschema", "dev123", "actuatorB");
  ActuatorLocator d("myschema", "dev456", "actuatorA");
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_FALSE(a == c);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == d);
  EXPECT_TRUE(a != d);
}

TEST(ActuatorLocatorTest, ToStringAndWriteCstr) {
  ActuatorLocator loc("myschema", "dev123", "actuatorA");
  std::string s = loc.toString();
  EXPECT_FALSE(s.empty());
  char buf[64];
  loc.write_cstr(buf);
  EXPECT_EQ(buf, s);
  EXPECT_EQ(s, "myschema:dev123/actuatorA");
}

}  // namespace roo_transceivers