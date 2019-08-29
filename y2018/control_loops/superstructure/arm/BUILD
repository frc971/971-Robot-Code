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
    deps = [
        ":demo_path",
        ":dynamics",
        ":ekf",
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
        ":ekf",
        ":generated_graph",
        ":trajectory",
        "//third_party/matplotlib-cpp",
        "@com_github_gflags_gflags//:gflags",
        "@org_tuxfamily_eigen//:eigen",
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
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "graph",
    srcs = ["graph.cc"],
    hdrs = ["graph.h"],
)

cc_test(
    name = "graph_test",
    srcs = ["graph_test.cc"],
    deps = [
        ":graph",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "arm",
    srcs = [
        "arm.cc",
    ],
    hdrs = [
        "arm.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":demo_path",
        ":ekf",
        ":generated_graph",
        ":graph",
        ":trajectory",
        "//frc971/zeroing",
        "//y2018:constants",
        "//y2018/control_loops/superstructure:superstructure_position_fbs",
        "//y2018/control_loops/superstructure:superstructure_status_fbs",
    ],
)

genrule(
    name = "generated_graph_genrule",
    outs = [
        "generated_graph.h",
        "generated_graph.cc",
    ],
    cmd = "$(location //y2018/control_loops/python:graph_codegen) $(OUTS)",
    tools = [
        "//y2018/control_loops/python:graph_codegen",
    ],
)

cc_library(
    name = "generated_graph",
    srcs = [
        "generated_graph.cc",
    ],
    hdrs = ["generated_graph.h"],
    copts = [
        "-O1",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":graph",
        ":trajectory",
    ],
)
