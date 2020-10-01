package(default_visibility = ["//visibility:public"])

load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "ring_buffer",
    hdrs = [
        "ring_buffer.h",
    ],
)

cc_test(
    name = "ring_buffer_test",
    srcs = [
        "ring_buffer_test.cc",
    ],
    deps = [
        ":ring_buffer",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "priority_queue",
    hdrs = [
        "priority_queue.h",
    ],
)

cc_test(
    name = "priority_queue_test",
    srcs = [
        "priority_queue_test.cc",
    ],
    deps = [
        ":priority_queue",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "sized_array",
    hdrs = [
        "sized_array.h",
    ],
    compatible_with = mcu_cpus,
)

cc_test(
    name = "sized_array_test",
    srcs = [
        "sized_array_test.cc",
    ],
    deps = [
        ":sized_array",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "resizeable_buffer",
    hdrs = [
        "resizeable_buffer.h",
    ],
    deps = [
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "resizeable_buffer_test",
    srcs = [
        "resizeable_buffer_test.cc",
    ],
    deps = [
        ":resizeable_buffer",
        "//aos/testing:googletest",
        "@com_google_absl//absl/types:span",
    ],
)
