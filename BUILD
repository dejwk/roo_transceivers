load("@roo_pb//:defs.bzl", "roo_pb_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

roo_pb_library(
    name = "proto",
    srcs = ["proto/roo_transceivers.proto"],
    options = ["proto/roo_transceivers.roo_pb.toml"],
    strip_import_prefix = "proto",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "roo_transceivers",
    srcs = glob(
        [
            "src/**/*.cpp",
            "src/**/*.h",
        ],
        exclude = [
            "test/**",
            "src/roo_transceivers.pb.h",
        ],
    ),
    includes = [
        "src",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":proto",
        "@roo_collections",
        "@roo_logging",
        "@roo_prefs",
        "@roo_testing//:arduino",
        "@roo_threads",
    ],
)

cc_test(
    name = "id_test",
    srcs = [
        "test/id_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    linkstatic = 1,
    deps = [
        ":roo_transceivers",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "transceiver_collection_test",
    srcs = [
        "test/transceiver_collection_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    linkstatic = 1,
    deps = [
        ":roo_transceivers",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "remote_server_test",
    srcs = [
        "test/remote_server_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    linkstatic = 1,
    deps = [
        ":roo_transceivers",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "proto_test",
    srcs = ["test/proto_test.cpp"],
    deps = [
        ":roo_transceivers",
        "@googletest//:gtest_main",
    ],
)
