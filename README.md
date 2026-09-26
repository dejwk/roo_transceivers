# roo_transceivers

Detection and aggregation of arbitrary transceiver devices, supporting arbitrary sensors and actuators.

## Host emulation

Host builds use the roo_testing 2.0 Arduino ESP32 profile. With Bazelisk 1.21
or newer, a plain command defaults to that profile and prints a notice:

    bazel test ...
    bazel test ... --config=asan
    bazel test ... --config=roo_testing_arduino_esp32

The files under .roo_testing/bazelrc/esp32 are vendored from roo_testing;
follow their canonical-source headers when refreshing them.

## Protocol generation

The protocol uses `roo_pb` and requires C++17. For now, keep `roo_pb` and
`roo_io` checkouts beside this repository: `MODULE.bazel` selects them with
local path overrides. PlatformIO's `library.json` also selects `../roo_pb`;
Arduino users should install these local libraries in their library directory.
Bazel applications consuming this library must set the same overrides in their
root module, since dependency modules' overrides do not propagate.

The `@roo_pb//:defs.bzl` `roo_pb_library` rule regenerates
`proto_generated/roo_transceivers.pb.h` whenever the schema,
`proto/roo_transceivers.roo_pb.toml`, or the compiler changes. The library uses
that build output, so no manual generation is needed for Bazel builds:

```sh
bazel build //:proto
bazel test //:proto_test //:remote_server_test //:transceiver_collection_test //:id_test
```

To refresh the checked-in `src/roo_transceivers.pb.h` used by Arduino and
PlatformIO, run `bash proto/generate.sh` from any working directory. Set
`ROO_PB_DIR` if the compiler checkout is elsewhere. Generation needs Python 3.11
or newer; `clang-format`, when available, formats the checked-in header.

The wire protocol keeps the existing field numbers and capacities. The C++ API
now uses `roo_transceivers::Descriptor`, `ClientMessage`, `ServerMessage`, and
`Quantity::kTemperature` (and other scoped quantity values). Replace direct
struct access with generated accessors such as `add_sensors()`, `set_id()`,
`sensors(i)`, `sensors_size()`, and `contents_case()`. Transports should use
`roo_pb::Serialize()` and `ParseFromArray()` for encoding and decoding.
