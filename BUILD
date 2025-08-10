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
        "@roo_collections",
        "@roo_logging",
        "@roo_prefs",
        "@roo_threads",
        "@roo_testing//:arduino",
        "@nanopb//:nanopb"
    ],
)

cc_test(
    name = "roo_transceivers_test",
    srcs = [
        "test/roo_transceivers_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    linkstatic = 1,
    deps = [
        ":roo_transceivers",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "roo_transceivers_collection_universe_test",
    srcs = [
        "test/roo_transceivers_collection_universe_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    linkstatic = 1,
    deps = [
        ":roo_transceivers",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "roo_transceivers_locator_test",
    srcs = [
        "test/roo_transceivers_locator_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    linkstatic = 1,
    deps = [
        ":roo_transceivers",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "roo_transceivers_remote_test",
    srcs = [
        "test/roo_transceivers_remote_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    linkstatic = 1,
    deps = [
        ":roo_transceivers",
        "@googletest//:gtest_main",
    ],
)
