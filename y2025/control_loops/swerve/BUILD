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
    deps = [
        "//frc971/control_loops/swerve:simplified_dynamics",
        "//y2025:constants",
    ],
)
