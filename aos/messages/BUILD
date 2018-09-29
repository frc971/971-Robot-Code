package(default_visibility = ["//visibility:public"])

cc_library(
    name = "messages",
    srcs = [
        "message.cc",
    ],
    hdrs = [
        "message.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos:byteorder",
        "//aos:macros",
        "//aos/time:time",
    ],
)
