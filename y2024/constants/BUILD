load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//frc971/constants:validator.bzl", "constants_json")
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
    name = "test_constants_unvalidated.json",
    src = "test_constants.jinja2.json",
    includes = glob([
        "test_data/*.json",
    ]) + [
        "common.jinja2",
        "common.json",
        "//y2024/constants/calib_files",
        "//y2024/control_loops/drivetrain:drivetrain_config.json",
        "//y2024/control_loops/superstructure/altitude:altitude_json",
        "//y2024/control_loops/superstructure/catapult:catapult_json",
        "//y2024/control_loops/superstructure/climber:climber_json",
        "//y2024/control_loops/superstructure/extend:extend_json",
        "//y2024/control_loops/superstructure/intake_pivot:intake_pivot_json",
        "//y2024/control_loops/superstructure/turret:turret_json",
        "//y2024/vision/maps",
    ],
    parameters = {},
    visibility = ["//visibility:public"],
)

jinja2_template(
    name = "constants_unvalidated.json",
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
        "//y2024/control_loops/superstructure/extend:extend_json",
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

cc_binary(
    name = "constants_formatter",
    srcs = ["constants_formatter.cc"],
    data = [":constants_unvalidated.json"],
    visibility = ["//visibility:public"],
    deps = [
        ":constants_list_fbs",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

constants_json(
    name = "constants_json",
    src = ":constants_unvalidated.json",
    out = "constants.json",
    year_tool = "//y2024/constants:constants_formatter",
)

constants_json(
    name = "test_constants_json",
    src = ":test_constants_unvalidated.json",
    out = "test_constants.json",
    year_tool = "//y2024/constants:constants_formatter",
)
