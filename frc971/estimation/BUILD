cc_library(
    name = "ekf",
    hdrs = ["ekf.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:c2d",
        "//frc971/control_loops:runge_kutta",
        "//frc971/control_loops/swerve:auto_diff_jacobian",
        "//frc971/control_loops/swerve:linearization_utils",
    ],
)

cc_test(
    name = "ekf_test",
    srcs = ["ekf_test.cc"],
    deps = [
        ":ekf",
        "//aos/testing:googletest",
    ],
)
