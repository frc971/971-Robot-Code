package(default_visibility = ["//visibility:public"])

cc_library(
    name = "voltage_cap",
    srcs = [
        "voltage_cap.cc",
    ],
    hdrs = [
        "voltage_cap.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "voltage_cap_test",
    srcs = [
        "voltage_cap_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":voltage_cap",
        "//aos/testing:googletest",
    ],
)
