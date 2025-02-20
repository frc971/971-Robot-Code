cc_library(
    name = "ekf",
    srcs = ["ekf.cc"],
    hdrs = ["ekf.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:c2d",
        "//frc971/control_loops:runge_kutta",
        "//frc971/control_loops/swerve:auto_diff_jacobian",
        "//frc971/control_loops/swerve:linearization_utils",
        "@com_google_absl//absl/flags:flag",
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

cc_library(
    name = "kf",
    srcs = ["kf.cc"],
    hdrs = ["kf.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:c2d",
        "@com_google_absl//absl/log",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
