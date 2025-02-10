cc_binary(
    name = "swerve_control_loops",
    srcs = [
        "swerve_control_loops_main.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":parameters",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/time",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops/swerve:swerve_control_loops",
        "//y2025/constants:constants_fbs",
    ],
)

cc_library(
    name = "parameters",
    hdrs = ["parameters.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/swerve:linear_velocity_controller",
        "//frc971/control_loops/swerve:simplified_dynamics",
        "//y2025:constants",
    ],
)

cc_binary(
    name = "swerve_control_loops_replay",
    srcs = ["swerve_control_loops_replay.cc"],
    data = [
        "//y2025:aos_config",
        "//y2025/constants:constants.json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":parameters",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//aos/network:team_number",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops/swerve:swerve_control_loops",
        "//y2025/constants:constants_fbs",
        "//y2025/constants:simulated_constants_sender",
    ],
)
