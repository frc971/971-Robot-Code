cc_library(
    name = "trajectory",
    srcs = [
        "trajectory.cc",
    ],
    hdrs = [
        "trajectory.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
        "//frc971/control_loops:dlqr",
        "//frc971/control_loops:jacobian",
        "//third_party/eigen",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = [
        "trajectory_test.cc",
    ],
    deps = [
        ":demo_path",
        ":dynamics",
        ":ekf",
        ":trajectory",
        "//aos/testing:googletest",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "dynamics",
    srcs = [
        "dynamics.cc",
    ],
    hdrs = [
        "dynamics.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:runge_kutta",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "demo_path",
    srcs = [
        "demo_path.cc",
    ],
    hdrs = ["demo_path.h"],
    deps = [":trajectory"],
)

cc_test(
    name = "dynamics_test",
    srcs = [
        "dynamics_test.cc",
    ],
    deps = [
        ":dynamics",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "trajectory_plot",
    srcs = [
        "trajectory_plot.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":demo_path",
        ":ekf",
        ":trajectory",
        "//third_party/eigen",
        "//third_party/matplotlib-cpp",
    ],
)

cc_library(
    name = "ekf",
    srcs = [
        "ekf.cc",
    ],
    hdrs = [
        "ekf.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
        "//frc971/control_loops:jacobian",
        "//third_party/eigen",
    ],
)
