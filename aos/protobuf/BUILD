package(default_visibility = ['//visibility:public'])

cc_library(
  name = "stack_arena",
  srcs = ["stack_arena.cc"],
  hdrs = ["stack_arena.h"],
  deps = [
    "//aos/common/logging:logging",
    "//third_party/protobuf",
  ],
)

cc_test(
  name = "stack_arena_test",
  srcs = ["stack_arena_test.cc"],
  deps = [
    ":stack_arena",
    "//aos/testing:test_logging",
    "//aos/common/logging:logging",
    "//aos/testing:googletest",
  ],
)
