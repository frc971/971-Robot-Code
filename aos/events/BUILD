
cc_library(
  name = "raw-event-loop",
  hdrs = ["raw-event-loop.h"],
  deps = [
    "//aos/common:time",
    "//aos/common:queues",
  ],
)

cc_library(
  name = "event-loop",
  hdrs = ["event-loop.h", "raw-event-loop.h"],
  srcs = ["event-loop-tmpl.h"],
  deps = [
    ":raw-event-loop",
    "//aos/common:time",
    "//aos/common:queues",
  ],
)

cc_library(
  name = "shm-event-loop",
  hdrs = ["shm-event-loop.h"],
  srcs = ["shm-event-loop.cc"],
  deps = [
    ":event-loop",
    "//aos/common:queues",
    "//aos/common/logging:logging",
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
  srcs = ["event-loop_param_test.cc"],
  hdrs = ["event-loop_param_test.h"],
  deps = [
    "event-loop",
    "//aos/testing:googletest",
  ],
  testonly = True,
)

cc_test(
  name = "simulated-event-loop_test",
  srcs = ["simulated-event-loop_test.cc"],
  deps = [
    "//aos/testing:googletest",
    ":event-loop_param_test",
    ":simulated-event-loop",
  ],
  testonly = True,
)

cc_library(
  name = "simulated-event-loop",
  hdrs = ["simulated-event-loop.h"],
  srcs = ["simulated-event-loop.cc"],
  deps = [
    ":event-loop",
    "//aos/common:queues",
  ],
)

