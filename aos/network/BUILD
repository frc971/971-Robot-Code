package(default_visibility = ["//visibility:public"])

cc_library(
    name = "team_number",
    srcs = [
        "team_number.cc",
    ],
    hdrs = [
        "team_number.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos:once",
        "//aos/logging",
        "//aos/util:string_to_num",
        "//aos:configuration",
    ],
)

cc_test(
    name = "team_number_test",
    srcs = [
        "team_number_test.cc",
    ],
    deps = [
        ":team_number",
        "//aos/testing:googletest",
    ],
)
