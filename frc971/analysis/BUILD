package(default_visibility = ["//visibility:public"])

load("@npm_bazel_typescript//:defs.bzl", "ts_library")
load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "nodejs_binary", "rollup_bundle")

py_binary(
    name = "plot_action",
    srcs = [
        "logentry.py",
        "logreader.py",
        "plot_action.py",
        "plotter.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":python_init",
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

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
        "//aos/events/logging:logger",
        "@com_github_google_glog//:glog",
        "@python_repo//:python3.5_lib",
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
        "//aos:configuration_ts_fbs",
        "//aos/network/www:demo_plot",
        "//aos/network/www:proxy",
        "//frc971/wpilib:imu_plotter",
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
