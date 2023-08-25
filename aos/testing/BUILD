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
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
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
        "@com_github_google_glog//:glog",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)
