package(default_visibility = ["//visibility:public"])

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
