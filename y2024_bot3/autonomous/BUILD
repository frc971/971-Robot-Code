package(default_visibility = ["//y2024_bot3:__subpackages__"])

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
        "paths/swerve-path.json",
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
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
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

cc_binary(
    name = "autonomous_main",
    srcs = ["autonomous_main.cc"],
    data = [
        ":trajectory_bfbs",
    ],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/control_loops/swerve:autonomous_controller",
        "//y2024_bot3/control_loops/superstructure:superstructure_goal_fbs",
    ],
)
