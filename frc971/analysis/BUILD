package(default_visibility = ["//visibility:public"])

load("@npm_bazel_typescript//:defs.bzl", "ts_library")
load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "nodejs_binary", "rollup_bundle")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//frc971:python_init"],
)

cc_binary(
    name = "py_log_reader.so",
    srcs = ["py_log_reader.cc"],
    linkshared = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "@com_github_google_glog//:glog",
        "@python_repo//:python3.7_lib",
    ],
)

py_test(
    name = "log_reader_test",
    srcs = ["log_reader_test.py"],
    data = [
        ":py_log_reader.so",
        "@sample_logfile//file",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos:configuration_fbs_python"],
)

py_proto_library(
    name = "plot_config_proto",
    srcs = ["plot_config.proto"],
    target_compatible_with = ["@platforms//os:linux"],
)

py_binary(
    name = "plot",
    srcs = ["plot.py"],
    data = [
        ":py_log_reader.so",
    ] + glob(["plot_configs/**"]),
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":plot_config_proto",
        ":python_init",
        "@matplotlib_repo//:matplotlib3",
    ],
)

py_test(
    name = "plot_test",
    srcs = ["plot_test.py"],
    data = [
        "@sample_logfile//file",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":plot"],
)

ts_library(
    name = "plot_index",
    srcs = ["plot_index.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":plot_data_utils",
        "//aos:configuration_ts_fbs",
        "//aos/network/www:demo_plot",
        "//aos/network/www:proxy",
        "//frc971/control_loops/drivetrain:down_estimator_plotter",
        "//frc971/control_loops/drivetrain:drivetrain_plotter",
        "//frc971/control_loops/drivetrain:robot_state_plotter",
        "//frc971/control_loops/drivetrain:spline_plotter",
        "//frc971/wpilib:imu_plotter",
        "//y2020/control_loops/superstructure:accelerator_plotter",
        "//y2020/control_loops/superstructure:finisher_plotter",
        "//y2020/control_loops/superstructure:hood_plotter",
    ],
)

rollup_bundle(
    name = "plot_index_bundle",
    enable_code_splitting = False,
    entry_point = "plot_index.ts",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":plot_index",
    ],
)

filegroup(
    name = "plotter_files",
    srcs = [
        "index.html",
        "plot_index_bundle.min.js",
    ],
)

sh_binary(
    name = "web_plotter",
    srcs = ["web_plotter.sh"],
    data = [
        ":plotter_files",
        "//aos/network:log_web_proxy_main",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

sh_binary(
    name = "live_web_plotter_demo",
    srcs = ["live_web_plotter_demo.sh"],
    data = [
        ":plotter_files",
        "//aos/network:web_proxy_main",
        "//aos/network/www:test_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "plot_data_fbs",
    srcs = [
        "plot_data.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_ts_library(
    name = "plot_data_ts_fbs",
    srcs = [
        "plot_data.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

ts_library(
    name = "plot_data_utils",
    srcs = ["plot_data_utils.ts"],
    visibility = ["//visibility:public"],
    deps = [
        ":plot_data_ts_fbs",
        "//aos:configuration_ts_fbs",
        "//aos/network/www:aos_plotter",
        "//aos/network/www:plotter",
        "//aos/network/www:proxy",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

aos_config(
    name = "plotter",
    src = "plotter_config.json",
    flatbuffers = [":plot_data_fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:config"],
)

cc_library(
    name = "in_process_plotter",
    srcs = ["in_process_plotter.cc"],
    hdrs = ["in_process_plotter.h"],
    copts = [
        "-DWEBRTC_POSIX",
    ],
    deps = [
        ":plot_data_fbs",
        "//aos/events:simulated_event_loop",
        "//aos/network:web_proxy",
    ],
)

cc_binary(
    name = "in_process_plotter_demo",
    srcs = ["in_process_plotter_demo.cc"],
    copts = [
        "-DWEBRTC_POSIX",
    ],
    data = [
        ":plotter",
        ":plotter_files",
    ],
    # Tagged manual until we either get the linker working with the current
    # WebRTC implementation or we get a new implementation.
    tags = ["manual"],
    deps = [
        ":in_process_plotter",
        "//aos:init",
    ],
)
