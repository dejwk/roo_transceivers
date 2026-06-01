# Programming guide

> This guide starts with the simplest local use case and then expands toward
> composition, remote access, and persistent bindings.

## Before you start

### Purpose

Use `roo_transceivers` when your application wants to treat a heterogeneous
set of hardware devices as one collection of named sensors and actuators.
It is useful when devices may appear or disappear at runtime, when the
application needs to inspect capabilities before using them, or when the code
that consumes measurements should not care whether the data comes from a local
driver, a bus adapter, or a remote peer.

Typical examples include a controller that discovers 1-Wire thermometers, a UI
that lets the user choose which relay drives which function, or a gateway that
mirrors devices from another microcontroller over a transport link.

The library is intentionally small at the center. Most applications only need
five concepts:

- A [Universe](../src/roo_transceivers/universe.h), which is a collection of
  devices.
- [Locators](../src/roo_transceivers/id.h), which identify devices, sensors,
  and actuators.
- A [descriptor](../src/roo_transceivers/descriptor.h), which says what a
  device exposes.
- A [Measurement](../src/roo_transceivers/measurement.h), which is a quantity,
  timestamp, and value.
- An [EventListener](../src/roo_transceivers/notification.h), which is
  notified when the device set changes or fresh readings arrive.

### The core model

At a high level, `roo_transceivers` models a device graph.

- A *device* is an addressable thing, such as a thermometer, relay board, or
  remote endpoint.
- A *sensor* is something you can read from that device.
- An *actuator* is something you can write to on that device.
- A *descriptor* answers "what does this device expose?".
- A *measurement* answers "what is the latest known reading?".
- A *universe* answers "what devices exist right now, and how do I address
  them?".

Once those pieces are in place, the rest of the library is mostly about how to
build universes conveniently and how to compose them.

## Foundations

### Locators and stable identity

Every operation in the library is keyed by a locator.

```cpp
#include "roo_transceivers.h"

using namespace roo_transceivers;

DeviceLocator device("demo-temp", "room");
SensorLocator sensor(device, "temperature");
ActuatorLocator actuator("demo-relay", "fan", "relay1");
```

The string representation is designed to be readable:

- `DeviceLocator` looks like `schema:device_id`.
- `SensorLocator` looks like `schema:device_id/sensor_id`.
- `ActuatorLocator` looks like `schema:device_id/actuator_id`.

Treat the schema as a namespace for a device family or discovery mechanism.
For example, a 1-Wire thermometer universe might use the schema `1-Wire`, and
a remote mirrored universe might use something tied to the remote device type
or upstream transport.

The most important rule is stability. A device id should continue to mean the
same physical or logical device across restarts. If a user binds "room
temperature" to a locator, that locator should still be meaningful after a
reboot.

### Descriptors and quantities

A device descriptor tells the application which sensors and actuators a device
has, along with the quantity represented by each endpoint.

```cpp
roo_transceivers_Descriptor descriptor = {};
if (universe.getDeviceDescriptor(device, descriptor)) {
  for (size_t i = 0; i < descriptor.sensors_count; ++i) {
    Serial.print("sensor id: ");
    Serial.println(descriptor.sensors[i].id);
  }
  for (size_t i = 0; i < descriptor.actuators_count; ++i) {
    Serial.print("actuator id: ");
    Serial.println(descriptor.actuators[i].id);
  }
}
```

The descriptor answers capability questions, not state questions. It tells you
that a device exposes `temperature` or `relay1`; it does not tell you the
current reading. That separation is deliberate.

The quantity enum describes what a floating-point value means. Two sensors may
have different ids but the same quantity. For example, several different
temperature sensors may all report
`roo_transceivers_Quantity_kTemperature`, while relays typically use
`roo_transceivers_Quantity_kBinaryState`.

In application code, the normal pattern is:

- enumerate devices,
- inspect the descriptor,
- decide which locator you care about,
- then read or write through that locator.

### Measurements and freshness

`read()` returns the latest known [Measurement](../src/roo_transceivers/measurement.h).
That measurement carries three pieces of information:

- the quantity,
- the timestamp at which the reading was taken,
- the floating-point value.

There are three important states to distinguish.

- `isInitial()` means there is no known reading. This usually means the
  locator did not resolve, or the implementation has never produced a reading
  for it.
- `isDefined()` means both the quantity and value are present.
- `isUnknown()` means the quantity and timestamp may be known, but the value is
  currently unavailable and stored as `NaN`.

That distinction matters. An unknown value is different from a missing sensor.
If a sensor exists but is still warming up, waiting on conversion, or unable to
sample right now, returning an unknown value is often more truthful than
pretending the sensor does not exist.

Also note that `read()` usually does not mean "go to hardware right now".
It means "return the latest known reading". The refresh step is represented
separately by `requestUpdate()`.

## Working with a universe

### First local universe

The easiest way to get started is to build a local universe from a few small
transceiver objects. The helper classes in
[transceiver_collection.h](../src/roo_transceivers/helpers/transceiver_collection.h)
are designed for exactly that.

The example below exposes a single temperature sensor.

```cpp
#include "roo_transceivers.h"
#include "roo_transceivers/helpers/transceiver_collection.h"

using namespace roo_transceivers;

float readTemperatureFromHardware();

class Thermometer : public SimpleSensor {
 public:
  Thermometer()
      : SimpleSensor(roo_transceivers_Quantity_kTemperature, "temperature") {}

  void sample() {
    temperature_c_ = readTemperatureFromHardware();
    notifyNewReadingsAvailable();
  }

 protected:
  float readFromSensor() const override { return temperature_c_; }

 private:
  float temperature_c_ = 21.5f;
};

Thermometer room_sensor;
TransceiverCollection universe;

void setup() {
  universe.add(DeviceLocator("demo-temp", "room"), &room_sensor);
}

void loop() {
  room_sensor.sample();

  Measurement m =
      universe.read(SensorLocator("demo-temp", "room", "temperature"));
  if (m.isDefined()) {
    Serial.println(m.value());
  }
}
```

This example already shows the intended workflow.

- `SimpleSensor` gives you the smallest possible device implementation.
- `TransceiverCollection` turns one or more device objects into a universe.
- The application reads through locators, not through the concrete class.

That last point is the main abstraction win. Once everything is behind a
universe, the calling code no longer needs to know where a reading came from.

### Discovering devices

Applications that do not hardcode locators should start by enumerating the
universe and reading descriptors.

```cpp
void dumpUniverse(Universe& universe) {
  universe.forEachDevice([&](const DeviceLocator& device) {
    roo_transceivers_Descriptor descriptor = {};
    if (!universe.getDeviceDescriptor(device, descriptor)) {
      return true;
    }

    Serial.println(device.toString().c_str());

    for (size_t i = 0; i < descriptor.sensors_count; ++i) {
      Serial.print("  sensor: ");
      Serial.println(descriptor.sensors[i].id);
    }

    for (size_t i = 0; i < descriptor.actuators_count; ++i) {
      Serial.print("  actuator: ");
      Serial.println(descriptor.actuators[i].id);
    }
    return true;
  });
}
```

The callback passed to `forEachDevice()` can stop enumeration early by
returning `false`. That is useful when you only need to find the first device
matching some condition.

Avoid assuming too much from device ids alone. The descriptor is the source of
truth for what the device actually exposes.

### Reading sensors and writing actuators

Once you know the locators you care about, using them is simple.

```cpp
SensorLocator temperature("demo-temp", "room", "temperature");
ActuatorLocator fan("demo-relay", "fan", "relay1");

Measurement m = universe.read(temperature);
if (m.isInitial()) {
  Serial.println("sensor not found");
} else if (m.isUnknown()) {
  Serial.println("sensor exists, but has no value yet");
} else if (m.value() > 26.0f) {
  if (!universe.write(fan, 1.0f)) {
    Serial.println("failed to switch the relay");
  }
}
```

Three rules are worth remembering.

- `read()` returns an initial measurement when the device or sensor does not
  exist.
- `write()` returns `false` when the actuator does not exist, the locator is
  invalid, or the target rejects the value.
- You should interpret values in the context of the descriptor's quantity.
  For example, binary state typically uses `0.0f` and `1.0f`.

### Polling and events

`requestUpdate()` exists because some universes need a separate refresh step.
For a cached remote universe, it may send a request to the peer. For a bus
adapter, it may start a hardware conversion. For a purely local helper class,
it may do nothing at all.

If your universe supports asynchronous updates, register an event listener.

```cpp
class Logger : public EventListener {
 public:
  void devicesChanged() override {
    Serial.println("device set changed");
  }

  void newReadingsAvailable() override {
    Serial.println("fresh readings are available");
  }
};

Logger logger;

void setup() {
  universe.addEventListener(&logger);
}

void loop() {
  universe.requestUpdate();
}
```

Use the two callbacks differently.

- `devicesChanged()` means locators or descriptors may have changed. Re-enumerate
  if your UI or binding logic depends on the current device set.
- `newReadingsAvailable()` means existing locators may now return fresher
  measurements.

### Missing devices, unknown values, and other practical rules

In a dynamic universe, a little discipline prevents subtle bugs.

- Treat locators as durable names, but treat descriptors as current state.
  A locator can survive across runs, while the descriptor behind it can change.
- Do not rely on enumeration order unless a specific universe documents one.
- If you cache a descriptor, invalidate that cache on `devicesChanged()`.
- When a sensor is optional or slow, distinguish `isUnknown()` from
  `isInitial()` in your UI and control logic.

## Building and integrating universes

### Implementing devices with helper classes

Most local device implementations should start with one of the helper types in
[transceiver_collection.h](../src/roo_transceivers/helpers/transceiver_collection.h).

- Use `SimpleSensor` when a device exposes exactly one sensor and no actuators.
- Use `SimpleTransceiver` when a device has a fixed descriptor with multiple
  sensors or actuators.
- Use `TransceiverCollection` when you want to register several local device
  objects and expose them as one universe.

`SimpleTransceiver` is the usual next step after `SimpleSensor`. It combines a
static descriptor with name lookup for sensors and actuators, then asks you to
implement `readFromSensor()` and `writeToActuator()` by index.

That is a good fit for devices such as relay boards, dimmers, or environmental
sensor modules with a stable endpoint layout.

### Adapting buses with `SimpleSensorUniverse` and `OneWireUniverse`

If your hardware source is naturally a whole bus rather than a collection of
pre-instantiated device objects, start one level higher with
[SimpleSensorUniverse](../src/roo_transceivers/helpers/simple_sensor_universe.h).

That helper is for the special but common case where:

- each discovered device exposes exactly one sensor,
- there are no actuators,
- the descriptor can be synthesized from the device locator.

In that model, the sensor id is the empty string `""`, because there is only
one sensor per device.

[OneWireUniverse](../src/roo_transceivers/onewire/onewire.h) is the built-in
example of this pattern.

```cpp
#include "roo_transceivers/onewire/onewire.h"

using namespace roo_transceivers;

roo_onewire::OneWire one_wire(/* bus setup */);
OneWireUniverse universe(one_wire);

void loop() {
  universe.requestUpdate();

  Measurement m =
      universe.read(SensorLocator("1-Wire", "28ff641d2c1603ab", ""));
  if (m.isDefined()) {
    Serial.println(m.value());
  }
}
```

`OneWireUniverse` uses the schema `1-Wire`, the thermometer ROM code as the
device id, and an empty sensor id. That is a good example of using locators to
encode stable discovery identity.

### Combining sources with `Multiverse`

Real applications often have more than one source of devices. You might have
some local helper objects, a 1-Wire bus, and a mirrored remote universe.

[Multiverse](../src/roo_transceivers/helpers/multiverse.h) combines them into
one universe.

```cpp
Multiverse all({&local_universe, &onewire_universe, &remote_universe});
```

Two details matter here.

- Enumeration simply walks the child universes in order.
- `getDeviceDescriptor()`, `read()`, and `write()` stop at the first child
  universe that handles the locator.

That means locator uniqueness matters. In practice, treat the schema as part of
your collision-avoidance strategy. If two child universes could expose the same
device id, give them different schemas.

### Mirroring a remote universe

The remote layer lets one microcontroller expose its universe to another.

- [UniverseServer](../src/roo_transceivers/remote/server.h) watches a source
  universe and sends snapshots and deltas.
- [UniverseClient](../src/roo_transceivers/remote/client.h) receives those
  updates and exposes a normal local `Universe` view.

The transport itself is abstracted behind channel interfaces. You provide a
`UniverseServerChannel` and `UniverseClientChannel` on top of whatever link or
messaging layer you are already using.

The important application-level point is that the consumer of the universe does
not need special logic for local versus remote data. Once the client has been
started with `begin()`, you enumerate it, inspect descriptors, read cached
measurements, and write actuators exactly as you would with a local universe.

`requestUpdate()` on the client side asks the server side to refresh the source
universe. That keeps the same mental model even when the data path is remote.

In most applications, the generated protocol messages are an implementation
detail of the channel layer, not something that application code manipulates
directly.

### Binding application roles to discovered devices

Discovery gives you dynamic locators. Bindings let you persist the application
meaning of those locators.

Suppose your UI lets the user choose which discovered sensor counts as "room
temperature". That choice should survive a reboot. The binding layer stores a
locator under an application-defined key, then lets the application read or
write through that stored binding.

```cpp
#include "roo_transceivers/binding/binding.h"
#include "roo_transceivers/binding/hal/defaults.h"

using namespace roo_transceivers;

SensorBinding room_temperature_binding(DefaultBindingStore(), 1);
BoundSensor room_temperature(universe, &room_temperature_binding);

void ensureDefaultBinding() {
  if (!room_temperature_binding.isBound()) {
    room_temperature_binding.bind(
        SensorLocator("demo-temp", "room", "temperature"));
  }
}

void loop() {
  Measurement m = room_temperature.read();
  if (m.isDefined()) {
    Serial.println(m.value());
  }
}
```

The built-in [DefaultBindingStore](../src/roo_transceivers/binding/hal/defaults.h)
uses the Arduino preferences-backed
[ArduinoPreferencesBindingStore](../src/roo_transceivers/binding/hal/arduino_prefs/arduino_prefs_store.h).
You can also provide your own [BindingStore](../src/roo_transceivers/binding/hal/store.h)
implementation.

Bindings store locators, not live readings. If the bound device disappears,
`BoundSensor::read()` returns an initial measurement until the device comes
back or the binding is changed.

### Choosing the right level of abstraction

If you are not sure where to start, use this rule of thumb.

- One read-only value per device: start with `SimpleSensor`.
- One fixed device type with several endpoints: use `SimpleTransceiver`.
- Several local device objects: expose them with `TransceiverCollection`.
- A bus of identical single-sensor devices: use `SimpleSensorUniverse`.
- Several independent universes: combine them with `Multiverse`.
- Another microcontroller's device graph: mirror it with
  `UniverseServer` and `UniverseClient`.
- User-selectable logical roles such as "main room temperature" or "fan
  relay": add bindings.

If you keep that ladder in mind, most `roo_transceivers` designs stay small and
composable.