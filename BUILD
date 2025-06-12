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
        "//lib/roo_collections",
        "//lib/roo_logging",
        "//lib/roo_prefs",
        "//roo_testing:arduino",
        "@nanopb//:nanopb"
    ],
)
