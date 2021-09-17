package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bitpacking",
    hdrs = [
        "bitpacking.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/GSL",
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

cc_library(
    name = "run_command",
    srcs = [
        "run_command.cc",
    ],
    hdrs = [
        "run_command.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
    ],
)

cc_test(
    name = "run_command_test",
    srcs = [
        "run_command_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":run_command",
        "//aos/logging",
        "//aos/testing:googletest",
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
        "//aos/testing:googletest",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//aos:python_init"],
)
