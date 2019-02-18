cc_library(
    name = "raw-event-loop",
    hdrs = ["raw-event-loop.h"],
    deps = [
        "//aos:queues",
        "//aos/time",
    ],
)

cc_library(
    name = "event-loop",
    srcs = ["event-loop-tmpl.h"],
    hdrs = [
        "event-loop.h",
        "raw-event-loop.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":raw-event-loop",
        "//aos:queues",
        "//aos/time",
    ],
)

cc_library(
    name = "shm-event-loop",
    srcs = ["shm-event-loop.cc"],
    hdrs = ["shm-event-loop.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":event-loop",
        "//aos:queues",
        "//aos/logging",
    ],
)

cc_test(
    name = "shm-event-loop_test",
    srcs = ["shm-event-loop_test.cc"],
    deps = [
        ":event-loop_param_test",
        ":shm-event-loop",
        "//aos/testing:test_shm",
    ],
)

cc_library(
    name = "event-loop_param_test",
    testonly = True,
    srcs = ["event-loop_param_test.cc"],
    hdrs = ["event-loop_param_test.h"],
    deps = [
        "event-loop",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "simulated-event-loop_test",
    testonly = True,
    srcs = ["simulated-event-loop_test.cc"],
    deps = [
        ":event-loop_param_test",
        ":simulated-event-loop",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "simulated-event-loop",
    srcs = ["simulated-event-loop.cc"],
    hdrs = ["simulated-event-loop.h"],
    deps = [
        ":event-loop",
        "//aos:queues",
        "//aos/logging",
    ],
)
