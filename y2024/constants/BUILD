load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:template.bzl", "jinja2_template")

cc_library(
    name = "simulated_constants_sender",
    srcs = ["simulated_constants_sender.cc"],
    hdrs = ["simulated_constants_sender.h"],
    data = [":test_constants.json"],
    visibility = ["//y2024:__subpackages__"],
    deps = [
        ":constants_fbs",
        ":constants_list_fbs",
        "//aos/events:simulated_event_loop",
        "//aos/testing:path",
        "//frc971/constants:constants_sender_lib",
    ],
)

jinja2_template(
    name = "test_constants.json",
    src = "test_constants.jinja2.json",
    includes = glob([
        "test_data/*.json",
    ]) + [
        "//y2024/control_loops/superstructure/intake_pivot:intake_pivot_json",
        "//y2024/control_loops/superstructure/climber:climber_json",
        "//y2024/control_loops/superstructure/catapult:catapult_json",
        "//y2024/control_loops/superstructure/altitude:altitude_json",
        "//y2024/control_loops/superstructure/turret:turret_json",
        "//y2024/control_loops/drivetrain:drivetrain_config.json",
        "//y2024/constants/calib_files",
        "common.json",
        "common.jinja2",
        "//y2024/vision/maps",
    ],
    parameters = {},
    visibility = ["//visibility:public"],
)

jinja2_template(
    name = "constants.json",
    src = "constants.jinja2.json",
    includes = [
        "7971.json",
        "971.json",
        "9971.json",
        "common.jinja2",
        "common.json",
        "//y2024/constants/calib_files",
        "//y2024/control_loops/drivetrain:drivetrain_config.json",
        "//y2024/control_loops/superstructure/altitude:altitude_json",
        "//y2024/control_loops/superstructure/catapult:catapult_json",
        "//y2024/control_loops/superstructure/climber:climber_json",
        "//y2024/control_loops/superstructure/intake_pivot:intake_pivot_json",
        "//y2024/control_loops/superstructure/turret:turret_json",
        "//y2024/vision/maps",
    ],
    parameters = {},
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "constants_fbs",
    srcs = ["constants.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_config_fbs",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:target_map_fbs",
        "//frc971/zeroing:constants_fbs",
    ],
)

static_flatbuffer(
    name = "constants_list_fbs",
    srcs = ["constants_list.fbs"],
    visibility = ["//visibility:public"],
    deps = [":constants_fbs"],
)

cc_binary(
    name = "constants_sender",
    srcs = ["constants_sender.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":constants_fbs",
        ":constants_list_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/testing:path",
        "//frc971/constants:constants_sender_lib",
    ],
)

cc_test(
    name = "constants_validator_test",
    srcs = ["constants_validator_test.cc"],
    data = [":constants.json"],
    visibility = ["//visibility:public"],
    deps = [
        ":constants_list_fbs",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
        "@com_github_google_glog//:glog",
    ],
)
