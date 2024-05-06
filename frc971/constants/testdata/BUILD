load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

exports_files([
    "test_constants.json",
    "syntax_error.json",
])

static_flatbuffer(
    name = "constants_list_fbs",
    srcs = ["constants_list.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":constants_data_fbs",
    ],
)

static_flatbuffer(
    name = "constants_data_fbs",
    srcs = ["constants_data.fbs"],
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
