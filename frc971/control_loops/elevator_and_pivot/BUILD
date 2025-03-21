cc_library(
    name = "inverse_kinematics",
    hdrs = ["inverse_kinematics.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "inverse_kinematics_test",
    srcs = ["inverse_kinematics_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":inverse_kinematics",
        "//aos/testing:googletest",
    ],
)
