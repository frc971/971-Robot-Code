load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//y2025:__subpackages__"])

filegroup(
    name = "binaries.stripped",
    srcs = [
        ":autonomous_main.stripped",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "binaries",
    srcs = [
        ":autonomous_main",
    ],
    visibility = ["//visibility:public"],
)

genrule(
    name = "genrule_trajectory",
    srcs = [
        "paths/L4-3-good.json",
    ],
    outs = [
        "trajectory.json",
    ],
    cmd = "$(location //frc971/control_loops/swerve/spline_ui:generate_solved_trajectory) $(SRCS) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//frc971/control_loops/swerve/spline_ui:generate_solved_trajectory",
    ],
)

genrule(
    name = "precompile_trajectory_flatbuffer",
    srcs = [
        "trajectory.json",
    ],
    outs = [
        "trajectory.bfbs",
    ],
    cmd = "$(location //frc971/control_loops/swerve:gen_trajectory_flatbuffer) $(SRCS) $(OUTS)",
    tools = [
        "//frc971/control_loops/swerve:gen_trajectory_flatbuffer",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "trajectory_bfbs",
    srcs = [
        "trajectory.bfbs",
    ],
)

static_flatbuffer(
    name = "autonomous_superstructure_goal_fbs",
    srcs = ["autonomous_superstructure_goal.fbs"],
    deps = [
        "//y2025/control_loops/superstructure:superstructure_goal_fbs",
    ],
)

cc_binary(
    name = "autonomous_main",
    srcs = ["autonomous_main.cc"],
    data = [
        ":trajectory_bfbs",
    ],
    deps = [
        ":autonomous_superstructure_goal_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/control_loops/swerve:autonomous_controller",
    ],
)

cc_binary(
    name = "autonomous_superstructure",
    srcs = ["autonomous_superstructure.cc"],
    deps = [
        ":autonomous_superstructure_goal_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//y2025/control_loops/superstructure:superstructure_goal_fbs",
    ],
)
