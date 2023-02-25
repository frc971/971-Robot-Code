cc_library(
    name = "arm",
    srcs = [
        "arm.cc",
    ],
    hdrs = [
        "arm.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":generated_graph",
        "//frc971/control_loops/double_jointed_arm:ekf",
        "//frc971/control_loops/double_jointed_arm:graph",
        "//frc971/zeroing",
        "//y2023:constants",
        "//y2023/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//y2023/control_loops/superstructure:superstructure_position_fbs",
        "//y2023/control_loops/superstructure:superstructure_status_fbs",
        "//y2023/control_loops/superstructure/arm:arm_constants",
        "//y2023/control_loops/superstructure/arm:trajectory",
        "//y2023/control_loops/superstructure/roll:roll_plants",
    ],
)

genrule(
    name = "generated_graph_genrule",
    outs = [
        "generated_graph.h",
        "generated_graph.cc",
    ],
    cmd = "$(location //y2023/control_loops/python:graph_codegen) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2023/control_loops/python:graph_codegen",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":arm_constants",
        "//frc971/control_loops/double_jointed_arm:graph",
        "//y2023/control_loops/superstructure/arm:trajectory",
        "//y2023/control_loops/superstructure/roll:roll_plants",
    ],
)

cc_library(
    name = "arm_constants",
    hdrs = ["arm_constants.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/double_jointed_arm:dynamics",
    ],
)

cc_binary(
    name = "arm_design",
    srcs = [
        "arm_design.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":arm_constants",
        "//aos:init",
        "//frc971/analysis:in_process_plotter",
        "//frc971/control_loops:dlqr",
        "//frc971/control_loops:jacobian",
        "//frc971/control_loops/double_jointed_arm:dynamics",
    ],
)

cc_library(
    name = "trajectory",
    srcs = ["trajectory.cc"],
    hdrs = ["trajectory.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/control_loops:binomial",
        "//frc971/control_loops:dlqr",
        "//frc971/control_loops:fixed_quadrature",
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops/double_jointed_arm:dynamics",
        "//frc971/control_loops/double_jointed_arm:ekf",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = ["trajectory_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":arm_constants",
        ":trajectory",
        "//aos/testing:googletest",
        "//y2023/control_loops/superstructure/roll:roll_plants",
    ],
)

cc_binary(
    name = "trajectory_plot",
    srcs = [
        "trajectory_plot.cc",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":arm_constants",
        ":generated_graph",
        ":trajectory",
        "//frc971/analysis:in_process_plotter",
        "//frc971/control_loops:binomial",
        "//frc971/control_loops:fixed_quadrature",
        "//frc971/control_loops/double_jointed_arm:ekf",
        "//y2023/control_loops/superstructure/roll:roll_plants",
        "@com_github_gflags_gflags//:gflags",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
