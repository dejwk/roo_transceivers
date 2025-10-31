cc_library(
    name = "fake",
    hdrs = ["roo_transceivers/fake_transceivers.h"],
    deps = [
        "//:roo_transceivers",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "relay",
    srcs = ["roo_transceivers/relay.cpp"],
    hdrs = ["roo_transceivers/relay.h"],
    deps = [
        ":fake",
        "@roo_testing//roo_testing/transducers/voltage",
#        "@com_github_google_glog//:glog",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "environmental_sensor",
    srcs = ["roo_transceivers/environmental_sensor.cpp"],
    hdrs = ["roo_transceivers/environmental_sensor.h"],
    deps = [
        ":fake",
        "@roo_testing//roo_testing/transducers/temperature",
    ],
    visibility = ["//visibility:public"],
)

