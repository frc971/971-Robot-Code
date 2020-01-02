package(default_visibility = ["//visibility:public"])

load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")

py_binary(
    name = "plot_action",
    srcs = [
        "logentry.py",
        "logreader.py",
        "plot_action.py",
        "plotter.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        ":python_init",
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    deps = ["//frc971:python_init"],
)

cc_binary(
    name = "py_log_reader.so",
    srcs = ["py_log_reader.cc"],
    linkshared = True,
    restricted_to = ["//tools:k8"],
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
    restricted_to = ["//tools:k8"],
    deps = ["//aos:configuration_fbs_python"],
)

py_proto_library(
    name = "plot_config_proto",
    srcs = ["plot_config.proto"],
)

py_binary(
    name = "plot",
    srcs = ["plot.py"],
    data = [
        ":py_log_reader.so",
    ] + glob(["plot_configs/**"]),
    restricted_to = ["//tools:k8"],
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
    restricted_to = ["//tools:k8"],
    deps = [":plot"],
)
