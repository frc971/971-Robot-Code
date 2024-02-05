load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:js.bzl", "ts_project")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")

package(default_visibility = ["//visibility:public"])

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
        "//third_party/python",
        "@com_github_google_glog//:glog",
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

static_flatbuffer(
    name = "plot_data_fbs",
    srcs = [
        "plot_data.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_ts_library(
    name = "plot_data_ts_fbs",
    srcs = [
        "plot_data.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

ts_project(
    name = "plot_data_utils",
    srcs = ["plot_data_utils.ts"],
    visibility = ["//visibility:public"],
    deps = [
        ":plot_data_ts_fbs",
        "//aos:configuration_ts_fbs",
        "//aos/network/www:aos_plotter",
        "//aos/network/www:plotter",
        "//aos/network/www:proxy",
        "@com_github_google_flatbuffers//reflection:reflection_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

aos_config(
    name = "plotter",
    src = "plotter_config.json",
    flatbuffers = [":plot_data_fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

cc_library(
    name = "in_process_plotter",
    srcs = ["in_process_plotter.cc"],
    hdrs = ["in_process_plotter.h"],
    data = [
        ":plotter",
        "//aos/analysis/cpp_plot:cpp_plot_files",
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
    deps = [
        ":in_process_plotter",
        "//aos:init",
    ],
)

cc_binary(
    name = "local_foxglove",
    srcs = ["local_foxglove.cc"],
    data = ["@foxglove_studio"],
    deps = [
        "//aos:init",
        "//aos/network:gen_embedded",
        "//aos/seasocks:seasocks_logger",
        "//third_party/seasocks",
    ],
)
