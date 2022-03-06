package(default_visibility = ["//y2020:__subpackages__"])

genrule(
    name = "genrule_turret",
    outs = [
        "turret_plant.h",
        "turret_plant.cc",
        "integral_turret_plant.h",
        "integral_turret_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:turret) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2020/control_loops/python:turret",
    ],
)

cc_library(
    name = "turret_plants",
    srcs = [
        "integral_turret_plant.cc",
        "turret_plant.cc",
    ],
    hdrs = [
        "integral_turret_plant.h",
        "turret_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "aiming",
    srcs = ["aiming.cc"],
    hdrs = ["aiming.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:flatbuffers",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:pose",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/aiming",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//y2020:constants",
        "//y2020/control_loops/drivetrain:drivetrain_base",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
    ],
)

cc_test(
    name = "aiming_test",
    srcs = ["aiming_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aiming",
        "//aos/testing:googletest",
    ],
)
