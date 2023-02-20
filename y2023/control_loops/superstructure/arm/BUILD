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
        "//frc971/control_loops/double_jointed_arm:demo_path",
        "//frc971/control_loops/double_jointed_arm:ekf",
        "//frc971/control_loops/double_jointed_arm:graph",
        "//frc971/control_loops/double_jointed_arm:trajectory",
        "//frc971/zeroing",
        "//y2023:constants",
        "//y2023/control_loops/superstructure:superstructure_position_fbs",
        "//y2023/control_loops/superstructure:superstructure_status_fbs",
        "//y2023/control_loops/superstructure/arm:arm_constants",
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
        "//frc971/control_loops/double_jointed_arm:trajectory",
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
