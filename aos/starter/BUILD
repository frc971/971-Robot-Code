load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")

exports_files(["roborio_irq_config.json"])

# This target is everything which should get deployed to the robot.
filegroup(
    name = "starter",
    srcs = [
        "aos_starter",
        "starter.sh",
        "starterd.stripped",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "starter_stripped",
    srcs = [
        "aos_starter.stripped",
        "starter.sh",
        "starterd.stripped",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "subprocess",
    srcs = ["subprocess.cc"],
    hdrs = ["subprocess.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":starter_fbs",
        ":starter_rpc_fbs",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/util:scoped_pipe",
        "//aos/util:top",
        "@com_github_google_glog//:glog",
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
        ":subprocess",
        "//aos:configuration",
        "//aos:macros",
        "//aos/events:shm_event_loop",
        "@com_github_google_glog//:glog",
    ],
)

aos_config(
    name = "multinode_pingpong_config",
    src = "multinode_pingpong.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        ":starter_rpc_fbs",
        ":starter_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/events:event_loop_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:message_bridge_server_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "subprocess_test",
    srcs = ["subprocess_test.cc"],
    data = [
        "//aos/events:pingpong_config",
    ],
    flaky = True,
    # The roborio compiler doesn't support <filesystem>.
    target_compatible_with =
        ["@platforms//os:linux"],
    deps = [
        ":subprocess",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_test(
    name = "starter_test",
    srcs = ["starter_test.cc"],
    data = [
        ":multinode_pingpong_config",
        "//aos/events:ping",
        "//aos/events:pingpong_config",
        "//aos/events:pong",
    ],
    # TODO(james): Fix tihs.
    flaky = True,
    linkopts = ["-lstdc++fs"],
    shard_count = 4,
    # The roborio compiler doesn't support <filesystem>.
    target_compatible_with =
        select({
            "//tools/platforms/hardware:roborio": ["@platforms//:incompatible"],
            "//conditions:default": ["@platforms//os:linux"],
        }),
    deps = [
        ":starter_rpc_lib",
        ":starterd_lib",
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/events:simulated_event_loop",
        "//aos/ipc_lib:event",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_binary(
    name = "starterd",
    srcs = ["starterd.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
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
    visibility = ["//visibility:public"],
    deps = [
        ":starter_fbs",
        ":starter_rpc_fbs",
        ":starterd_lib",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_binary(
    name = "aos_starter",
    srcs = ["starter_cmd.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":starter_rpc_lib",
        "//aos/time",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
    ],
)

flatbuffer_cc_library(
    name = "starter_fbs",
    srcs = ["starter.fbs"],
    gen_reflections = True,
    includes = ["//aos/util:process_info_fbs_includes"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "kthread_fbs",
    srcs = ["kthread.fbs"],
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

py_binary(
    name = "starter_demo",
    srcs = ["starter_demo.py"],
    args = [
        "$(rootpath :starterd)",
        "\"$(rootpaths //aos/events:pingpong_config)\"",
        "$(rootpath //aos/events:ping)",
        "$(rootpath //aos/events:pong)",
        "$(rootpath :aos_starter)",
        "$(rootpath //aos:aos_dump)",
        "$(rootpath //aos/events/logging:logger_main)",
        "$(rootpath //aos/events:aos_timing_report_streamer)",
    ],
    data = [
        ":aos_starter",
        ":starterd",
        "//aos:aos_dump",
        "//aos/events:aos_timing_report_streamer",
        "//aos/events:ping",
        "//aos/events:pingpong_config",
        "//aos/events:pong",
        "//aos/events/logging:logger_main",
    ],
)

cc_library(
    name = "irq_affinity_lib",
    srcs = ["irq_affinity_lib.cc"],
    hdrs = ["irq_affinity_lib.h"],
    deps = [
        "//aos/scoped:scoped_fd",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "irq_affinity",
    srcs = [
        "irq_affinity.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":irq_affinity_lib",
        ":kthread_fbs",
        "//aos:init",
        "//aos:realtime",
        "//aos/events:shm_event_loop",
        "//aos/util:top",
    ],
)

cc_test(
    name = "irq_affinity_lib_test",
    srcs = ["irq_affinity_lib_test.cc"],
    deps = [
        ":irq_affinity_lib",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "mock_starter",
    srcs = ["mock_starter.cc"],
    hdrs = ["mock_starter.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:simulated_event_loop",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
        "//aos/starter:starter_rpc_lib",
    ],
)
