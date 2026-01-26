load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "fake",
    hdrs = ["roo_transceivers/fake_transceivers.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//:roo_transceivers",
    ],
)

cc_library(
    name = "relay",
    srcs = ["roo_transceivers/relay.cpp"],
    hdrs = ["roo_transceivers/relay.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":fake",
        "@roo_testing//roo_testing/transducers/voltage",
        #        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "environmental_sensor",
    srcs = ["roo_transceivers/environmental_sensor.cpp"],
    hdrs = ["roo_transceivers/environmental_sensor.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":fake",
        "@roo_testing//roo_testing/transducers/temperature",
    ],
)
