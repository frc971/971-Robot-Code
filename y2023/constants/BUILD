load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("@aspect_bazel_lib//lib:run_binary.bzl", "run_binary")
load("//tools/build_rules:template.bzl", "jinja2_template")

cc_library(
    name = "simulated_constants_sender",
    srcs = ["simulated_constants_sender.cc"],
    hdrs = ["simulated_constants_sender.h"],
    data = [":test_constants.json"],
    visibility = ["//y2023:__subpackages__"],
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
        ":wrist_common_zeroing.json",
        "//y2023/control_loops/superstructure/wrist:wrist_json",
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
        "common.json",
        ":scoring_map",
        ":wrist_common_zeroing.json",
        "//y2023/control_loops/superstructure/wrist:wrist_json",
        "//y2023/vision/calib_files",
        "//y2023/vision/maps",
    ],
    parameters = {},
    visibility = ["//visibility:public"],
)

filegroup(
    name = "scoring_map",
    srcs = [
        "relative_scoring_map.json",
        "scoring_map.json",
    ],
    visibility = ["//y2023:__subpackages__"],
)

static_flatbuffer(
    name = "constants_fbs",
    srcs = ["constants.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops:state_feedback_loop_fbs",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:target_map_fbs",
        "//frc971/zeroing:constants_fbs",
        "//y2023/localizer:scoring_map_fbs",
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
    name = "json_codegen",
    srcs = ["json_codegen.cc"],
    deps = [
        "//aos/util:file",
        "//y2023:constants",
    ],
)

run_binary(
    name = "wrist_json_codegen",
    outs = ["wrist_common_zeroing.json"],
    args = ["$(location :wrist_common_zeroing.json)"],
    tool = ":json_codegen",
)
