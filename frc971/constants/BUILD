cc_library(
    name = "constants_sender_lib",
    hdrs = [
        "constants_sender_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:flatbuffer_merge",
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/network:team_number",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "constants_sender_example",
    srcs = [
        "constants_sender_example.cc",
    ],
    data = [
        "//frc971/constants/testdata:aos_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/constants/testdata:constants_data_fbs",
        "//frc971/constants/testdata:constants_list_fbs",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "constants_sender_test",
    srcs = [
        "constants_sender_test.cc",
    ],
    data = [
        "//frc971/constants/testdata:aos_config",
        "//frc971/constants/testdata:syntax_error.json",
        "//frc971/constants/testdata:test_constants.json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":constants_sender_lib",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//frc971/constants/testdata:constants_data_fbs",
        "//frc971/constants/testdata:constants_list_fbs",
        "@com_github_google_glog//:glog",
    ],
)
