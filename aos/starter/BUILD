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
    deps = [
        "//aos:init",
        "//aos:once",
        "//aos:unique_malloc_ptr",
        "//aos/libc:aos_strsignal",
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/time",
        "//aos/util:run_command",
        "//third_party/libevent",
    ],
)
