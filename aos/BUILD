filegroup(
    name = "prime_binaries",
    srcs = [
        "//aos/common/logging:log_displayer",
        "//aos/common/logging:log_streamer",
        "//aos/linux_code:core",
        "//aos/linux_code/starter",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_start_binaries",
    srcs = [
        "//aos/common/logging:binary_log_writer",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_binaries_stripped",
    srcs = [
        # starter is hard coded to look for a non-stripped core...
        "//aos/linux_code:core",
        "//aos/common/logging:log_streamer.stripped",
        "//aos/common/logging:log_displayer.stripped",
        "//aos/linux_code/starter",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_start_binaries_stripped",
    srcs = [
        "//aos/common/logging:binary_log_writer.stripped",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "once",
    srcs = [
        "once-tmpl.h",
    ],
    hdrs = [
        "once.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/common:gtest_prod",
        "//aos/common:type_traits",
    ],
)

cc_test(
    name = "once_test",
    srcs = [
        "once_test.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:once",
        "//aos/testing:googletest",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)
