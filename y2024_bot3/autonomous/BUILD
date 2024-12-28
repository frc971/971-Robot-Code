load("//frc971/downloader:downloader.bzl", "aos_downloader_dir")

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
        "paths/slow-loop.json",
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

filegroup(
    name = "trajectory_jsons",
    srcs = [
        "trajectory.json",
    ],
)

aos_downloader_dir(
    name = "trajectory",
    srcs = [
        ":trajectory_jsons",
    ],
    dir = "trajectory",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "autonomous_main",
    srcs = ["autonomous_main.cc"],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/control_loops/swerve:autonomous_controller",
        "//y2024_bot3/control_loops/superstructure:superstructure_goal_fbs",
    ],
)
