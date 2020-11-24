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
