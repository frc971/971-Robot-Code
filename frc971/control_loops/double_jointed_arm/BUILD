cc_library(
    name = "trajectory",
    srcs = [
        "trajectory.cc",
    ],
    hdrs = [
        "trajectory.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
        "//aos/logging",
        "//frc971/control_loops:dlqr",
        "//frc971/control_loops:jacobian",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = [
        "trajectory_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":demo_path",
        ":dynamics",
        ":ekf",
        ":test_constants",
        ":trajectory",
        "//aos/testing:googletest",
        "@org_tuxfamily_eigen//:eigen",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:runge_kutta",
        "@com_github_gflags_gflags//:gflags",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "demo_path",
    srcs = [
        "demo_path.cc",
    ],
    hdrs = ["demo_path.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [":trajectory"],
)

cc_test(
    name = "dynamics_test",
    srcs = [
        "dynamics_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":dynamics",
        ":test_constants",
        "//aos/testing:googletest",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
        "//frc971/control_loops:jacobian",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "graph",
    srcs = ["graph.cc"],
    hdrs = ["graph.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "graph_test",
    srcs = ["graph_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":graph",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "test_constants",
    hdrs = ["test_constants.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
    ],
)
