load("//tools/build_rules:clean_dep.bzl", "aos_repo_name")
load("//tools/rust:defs.bzl", "rust_library")

cc_library(
    name = "googletest",
    testonly = True,
    srcs = [
        "gtest_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/testing:tmpdir",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "test_logging",
    testonly = True,
    srcs = [
        "test_logging.cc",
    ],
    hdrs = [
        "test_logging.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":googletest",
        "//aos/logging",
        "//aos/stl_mutex",
        "@com_google_absl//absl/base",
    ],
)

cc_test(
    name = "test_logging_test",
    srcs = [
        "test_logging_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":googletest",
        ":test_logging",
        "//aos/logging",
    ],
)

cc_library(
    name = "prevent_exit",
    testonly = True,
    srcs = [
        "prevent_exit.cc",
    ],
    hdrs = [
        "prevent_exit.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "test_shm",
    testonly = True,
    srcs = [
        "test_shm.cc",
    ],
    hdrs = [
        "test_shm.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":test_logging",
        "//aos/ipc_lib:shared_mem",
        "//aos/logging",
    ],
)

cc_library(
    name = "random_seed",
    testonly = True,
    srcs = [
        "random_seed.cc",
    ],
    hdrs = [
        "random_seed.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "tmpdir",
    testonly = True,
    srcs = ["tmpdir.cc"],
    hdrs = ["tmpdir.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/ipc_lib:shm_base",
    ],
)

cc_library(
    name = "flatbuffer_eq",
    testonly = True,
    hdrs = [
        "flatbuffer_eq.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "path",
    srcs = ["path.cc"],
    hdrs = ["path.h"],
    local_defines = ["AOS_REPO_NAME=\\\"" + aos_repo_name() + "\\\""],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)

rust_library(
    name = "path_rs",
    srcs = ["path.rs"],
    crate_name = "aos_testing_path",
    rustc_env = {"AOS_REPO_NAME": aos_repo_name()},
    visibility = ["//visibility:public"],
)

rust_library(
    name = "aos_rs",
    testonly = True,
    srcs = ["aos.rs"],
    crate_name = "aos",
    gen_docs = False,
    gen_doctests = False,
    visibility = ["//visibility:public"],
    deps = [
        ":path_rs",
        "//aos:aos_rs",
        "//aos:test_init_rs",
    ],
)
