
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
    "//aos/common:queues",
    "//aos/vision/events:intrusive_free_list",
    ":event-loop",
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
