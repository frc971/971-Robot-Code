load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

static_flatbuffer(
    name = "flywheel_controller_status_fbs",
    srcs = [
        "flywheel_controller_status.fbs",
    ],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "flywheel_controller_status_ts_fbs",
    srcs = [
        "flywheel_controller_status.fbs",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "flywheel_test_plant",
    hdrs = [
        "flywheel_test_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":flywheel_controller",
    ],
)

cc_library(
    name = "flywheel_controller",
    srcs = [
        "flywheel_controller.cc",
    ],
    hdrs = [
        "flywheel_controller.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":flywheel_controller_status_fbs",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:profiled_subsystem",
    ],
)

cc_test(
    name = "flywheel_controller_test",
    srcs = ["flywheel_controller_test.cc"],
    data = [
        ":flywheel_controller_test_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":flywheel_controller",
        ":flywheel_controller_test_plants",
        ":flywheel_test_plant",
        "//aos/testing:googletest",
        "//frc971/control_loops:control_loop_test",
    ],
)

aos_config(
    name = "flywheel_controller_test_config",
    src = "flywheel_controller_test_config_source.json",
    flatbuffers = [
        "//frc971/input:joystick_state_fbs",
        "//frc971/input:robot_state_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/events:event_loop_fbs",
        ":flywheel_controller_status_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

genrule(
    name = "genrule_flywheel_controller_test",
    outs = [
        "flywheel_controller_test_plant.h",
        "flywheel_controller_test_plant.cc",
        "integral_flywheel_controller_test_plant.h",
        "integral_flywheel_controller_test_plant.cc",
    ],
    cmd = "$(location //frc971/control_loops/python:flywheel_controller_test) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//frc971/control_loops/python:flywheel_controller_test",
    ],
)

cc_library(
    name = "flywheel_controller_test_plants",
    srcs = [
        "flywheel_controller_test_plant.cc",
        "integral_flywheel_controller_test_plant.cc",
    ],
    hdrs = [
        "flywheel_controller_test_plant.h",
        "integral_flywheel_controller_test_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
