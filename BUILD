load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "roo_transceivers",
    srcs = glob(
        [
            "src/**/*.cpp",
            "src/**/*.c",
            "src/**/*.h",
        ],
        exclude = ["test/**"],
    ),
    includes = [
        "src",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@nanopb",
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
