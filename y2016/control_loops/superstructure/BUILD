package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "superstructure_goal_fbs",
    srcs = [
        "superstructure_goal.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "superstructure_position_fbs",
    srcs = [
        "superstructure_position.fbs",
    ],
    gen_reflections = 1,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
    ],
)

flatbuffer_cc_library(
    name = "superstructure_output_fbs",
    srcs = [
        "superstructure_output.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "superstructure_status_fbs",
    srcs = [
        "superstructure_status.fbs",
    ],
    gen_reflections = 1,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
    ],
)

genrule(
    name = "genrule_intake",
    outs = [
        "intake_plant.h",
        "intake_plant.cc",
        "integral_intake_plant.h",
        "integral_intake_plant.cc",
    ],
    cmd = "$(location //y2016/control_loops/python:intake) $(OUTS)",
    tools = [
        "//y2016/control_loops/python:intake",
    ],
    visibility = ["//visibility:private"],
)

genrule(
    name = "genrule_arm",
    outs = [
        "arm_plant.h",
        "arm_plant.cc",
        "integral_arm_plant.h",
        "integral_arm_plant.cc",
    ],
    cmd = "$(location //y2016/control_loops/python:arm) $(OUTS)",
    tools = [
        "//y2016/control_loops/python:arm",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "superstructure_plants",
    srcs = [
        "arm_plant.cc",
        "intake_plant.cc",
        "integral_arm_plant.cc",
        "integral_intake_plant.cc",
    ],
    hdrs = [
        "arm_plant.h",
        "intake_plant.h",
        "integral_arm_plant.h",
        "integral_intake_plant.h",
    ],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "superstructure_lib",
    srcs = [
        "superstructure.cc",
        "superstructure_controls.cc",
    ],
    hdrs = [
        "superstructure.h",
        "superstructure_controls.h",
    ],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_plants",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:math",
        "//aos/controls:control_loop",
        "//aos/util:trapezoid_profile",
        "//frc971/control_loops:profiled_subsystem",
        "//frc971/control_loops:simple_capped_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/zeroing",
        "//y2016:constants",
        "//y2016/queues:ball_detector_fbs",
    ],
)

cc_test(
    name = "superstructure_lib_test",
    srcs = [
        "superstructure_lib_test.cc",
    ],
    data = ["//y2016:config.json"],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_lib",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:math",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//aos/time",
        "//frc971/control_loops:position_sensor_sim",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

cc_binary(
    name = "superstructure",
    srcs = [
        "superstructure_main.cc",
    ],
    deps = [
        ":superstructure_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
