load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

# This target is everything which should get deployed to the robot.
filegroup(
    name = "starter",
    srcs = [
        "starter.sh",
        "starter_exe",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "starter_exe",
    srcs = [
        "starter.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos:unique_malloc_ptr",
        "//aos/libc:aos_strsignal",
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/time",
        "//aos/util:run_command",
        "//third_party/libevent",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/base",
    ],
)

cc_library(
    name = "starterd_lib",
    srcs = ["starterd_lib.cc"],
    hdrs = ["starterd_lib.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":starter_fbs",
        ":starter_rpc_fbs",
        "//aos:configuration",
        "//aos:macros",
        "//aos/events:shm_event_loop",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "starter_test",
    srcs = ["starter_test.cc"],
    data = [
        "//aos/events:ping",
        "//aos/events:pingpong_config",
        "//aos/events:pong",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":starter_rpc_lib",
        ":starterd_lib",
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/testing:googletest",
        "//aos/testing:tmpdir",
    ],
)

cc_binary(
    name = "starterd",
    srcs = ["starterd.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":starterd_lib",
        "//aos:init",
    ],
)

cc_library(
    name = "starter_rpc_lib",
    srcs = ["starter_rpc_lib.cc"],
    hdrs = ["starter_rpc_lib.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":starter_fbs",
        ":starter_rpc_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_binary(
    name = "starter_cmd",
    srcs = ["starter_cmd.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":starter_rpc_lib",
        "@com_github_google_glog//:glog",
    ],
)

flatbuffer_cc_library(
    name = "starter_fbs",
    srcs = ["starter.fbs"],
    gen_reflections = True,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "starter_rpc_fbs",
    srcs = ["starter_rpc.fbs"],
    gen_reflections = True,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
