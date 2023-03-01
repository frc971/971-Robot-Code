package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ring_buffer",
    hdrs = [
        "ring_buffer.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "ring_buffer_test",
    srcs = [
        "ring_buffer_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":ring_buffer",
        "//aos/testing:googletest",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "priority_queue",
    hdrs = [
        "priority_queue.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "priority_queue_test",
    srcs = [
        "priority_queue_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
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
    deps = [
        "@com_google_absl//absl/container:inlined_vector",
    ],
)

cc_test(
    name = "sized_array_test",
    srcs = [
        "sized_array_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sized_array",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "error_list",
    hdrs = [
        "error_list.h",
    ],
    deps = [
        ":sized_array",
        "//aos:flatbuffers",
    ],
)

cc_test(
    name = "error_list_test",
    srcs = [
        "error_list_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":error_list",
        "//aos:json_to_flatbuffer_fbs",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "resizeable_buffer",
    hdrs = [
        "resizeable_buffer.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "resizeable_buffer_test",
    srcs = [
        "resizeable_buffer_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":resizeable_buffer",
        "//aos/testing:googletest",
        "@com_google_absl//absl/types:span",
    ],
)
