load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "shifter_hall_effect",
    hdrs = [
        "shifter_hall_effect.h",
    ],
)

cc_library(
    name = "constants",
    hdrs = [
        "constants.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "can_configuration_fbs",
    srcs = [
        ":can_configuration.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)
