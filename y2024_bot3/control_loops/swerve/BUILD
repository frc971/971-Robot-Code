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
        "//y2024_bot3/constants:constants_fbs",
    ],
)

cc_library(
    name = "parameters",
    hdrs = ["parameters.h"],
    deps = [
        "//frc971/control_loops/swerve:simplified_dynamics",
        "//y2024_bot3:constants",
    ],
)

cc_binary(
    name = "swerve_control_loops_replay",
    srcs = ["swerve_control_loops_replay.cc"],
    data = [
        "//y2024_bot3:aos_config",
        "//y2024_bot3/constants:constants.json",
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
        "//y2024_bot3/constants:constants_fbs",
        "//y2024_bot3/constants:simulated_constants_sender",
    ],
)
