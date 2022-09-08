load("//aos:config.bzl", "aos_config")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

exports_files(["test_constants.json"])

flatbuffer_cc_library(
    name = "constants_list_fbs",
    srcs = ["constants_list.fbs"],
    gen_reflections = 1,
    includes = [
        ":constants_data_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "constants_data_fbs",
    srcs = ["constants_data.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

aos_config(
    name = "aos_config",
    src = "test_config.json",
    flatbuffers = [
        "//frc971/constants/testdata:constants_data_fbs",
        "//frc971/constants/testdata:constants_list_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
    ],
)
