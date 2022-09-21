load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("config_validator_macro.bzl", "config_validator_rule")

package(default_visibility = ["//visibility:public"])

config_validator_rule(
    name = "config_validator_test",
    config = "//aos/events:pingpong_config",
)

cc_library(
    name = "bitpacking",
    hdrs = [
        "bitpacking.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "bitpacking_test",
    srcs = [
        "bitpacking_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":bitpacking",
        "//aos/testing:googletest",
    ],
)

py_library(
    name = "py_trapezoid_profile",
    srcs = [
        "trapezoid_profile.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":python_init",
    ],
)

cc_binary(
    name = "log_to_mcap",
    srcs = ["log_to_mcap.cc"],
    deps = [
        ":mcap_logger",
        "//aos:init",
        "//aos/events/logging:log_reader",
        "//frc971/control_loops:control_loops_fbs",
    ],
)

cc_library(
    name = "error_counter",
    hdrs = ["error_counter.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "error_counter_test",
    srcs = ["error_counter_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":error_counter",
        "//aos:flatbuffers",
        "//aos/events:event_loop_fbs",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "mcap_logger",
    srcs = ["mcap_logger.cc"],
    hdrs = ["mcap_logger.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration_fbs",
        "//aos:configuration_schema",
        "//aos:fast_string_builder",
        "//aos:flatbuffer_utils",
        "//aos/events:event_loop",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_binary(
    name = "generate_test_log",
    testonly = True,
    srcs = ["generate_test_log.cc"],
    data = ["//aos/events:pingpong_config"],
    deps = [
        "//aos:configuration",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/testing:path",
    ],
)

py_test(
    name = "log_to_mcap_test",
    srcs = ["log_to_mcap_test.py"],
    args = [
        "--log_to_mcap",
        "$(location :log_to_mcap)",
        "--mcap",
        "$(location @com_github_foxglove_mcap_mcap//file)",
        "--generate_log",
        "$(location :generate_test_log)",
    ],
    data = [
        ":generate_test_log",
        ":log_to_mcap",
        "@com_github_foxglove_mcap_mcap//file",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
)

cc_test(
    name = "mcap_logger_test",
    srcs = ["mcap_logger_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":mcap_logger",
        "//aos/testing:googletest",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_library(
    name = "math",
    hdrs = ["math.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "math_test",
    srcs = ["math_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":math",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "death_test_log_implementation",
    hdrs = [
        "death_test_log_implementation.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging:implementations",
    ],
)

cc_library(
    name = "phased_loop",
    srcs = [
        "phased_loop.cc",
    ],
    hdrs = [
        "phased_loop.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/time",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "log_interval",
    hdrs = [
        "log_interval.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
        "//aos/time",
    ],
)

cc_library(
    name = "trapezoid_profile",
    srcs = [
        "trapezoid_profile.cc",
    ],
    hdrs = [
        "trapezoid_profile.h",
    ],
    linkopts = [
        "-lm",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
        "//aos/time",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "trapezoid_profile_test",
    srcs = [
        "trapezoid_profile_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":trapezoid_profile",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "wrapping_counter",
    srcs = [
        "wrapping_counter.cc",
    ],
    hdrs = [
        "wrapping_counter.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "wrapping_counter_test",
    srcs = [
        "wrapping_counter_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":wrapping_counter",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "options",
    hdrs = [
        "options.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "options_test",
    srcs = [
        "options_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":options",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "compiler_memory_barrier",
    hdrs = [
        "compiler_memory_barrier.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "global_factory",
    hdrs = [
        "global_factory.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "global_factory_test",
    srcs = [
        "global_factory_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":global_factory",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "phased_loop_test",
    srcs = [
        "phased_loop_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":phased_loop",
        "//aos/testing:googletest",
        "//aos/time",
    ],
)

cc_library(
    name = "file",
    srcs = [
        "file.cc",
    ],
    hdrs = [
        "file.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/scoped:scoped_fd",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "file_test",
    size = "small",
    srcs = [
        "file_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":file",
        "//aos:realtime",
        "//aos/testing:googletest",
    ],
)

flatbuffer_cc_library(
    name = "process_info_fbs",
    srcs = ["process_info.fbs"],
    gen_reflections = True,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "top",
    srcs = ["top.cc"],
    hdrs = ["top.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":process_info_fbs",
        "//aos/containers:ring_buffer",
        "//aos/events:event_loop",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "top_test",
    srcs = ["top_test.cc"],
    data = [
        "//aos/events:pingpong_config",
    ],
    flaky = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":top",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_library(
    name = "scoped_pipe",
    srcs = ["scoped_pipe.cc"],
    hdrs = ["scoped_pipe.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "scoped_pipe_test",
    srcs = ["scoped_pipe_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":scoped_pipe",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "crc32",
    srcs = ["crc32.cc"],
    hdrs = ["crc32.h"],
    deps = [
        "@com_google_absl//absl/types:span",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//aos:python_init"],
)

cc_library(
    name = "threaded_queue",
    hdrs = [
        "threaded_queue.h",
        "threaded_queue_tmpl.h",
    ],
    deps = [
        "//aos:condition",
        "//aos/mutex",
    ],
)

cc_test(
    name = "threaded_queue_test",
    srcs = ["threaded_queue_test.cc"],
    deps = [
        ":threaded_queue",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "config_validator",
    testonly = True,
    srcs = ["config_validator.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//aos/testing:googletest",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)
