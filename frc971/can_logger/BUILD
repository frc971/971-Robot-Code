load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

cc_binary(
    name = "can_logger",
    srcs = [
        "can_logger_main.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":can_logger_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/time",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "can_logger_lib",
    srcs = [
        "can_logger.cc",
        "can_logger.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":can_logging_fbs",
        "//aos/events:shm_event_loop",
        "//aos/scoped:scoped_fd",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

static_flatbuffer(
    name = "can_logging_fbs",
    srcs = [
        "can_logging.fbs",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "log_to_asc",
    srcs = [
        "log_to_asc.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":asc_logger",
        ":can_logging_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events/logging:log_reader",
        "//aos/time",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "asc_logger",
    srcs = [
        "asc_logger.cc",
    ],
    hdrs = [
        "asc_logger.h",
    ],
    deps = [
        ":can_logging_fbs",
        "//aos/events:event_loop",
        "//aos/scoped:scoped_fd",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)
