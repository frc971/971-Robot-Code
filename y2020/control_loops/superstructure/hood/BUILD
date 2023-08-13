package(default_visibility = ["//y2020:__subpackages__"])

genrule(
    name = "genrule_hood",
    outs = [
        "hood_plant.h",
        "hood_plant.cc",
        "integral_hood_plant.h",
        "integral_hood_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:hood) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2020/control_loops/python:hood",
    ],
)

cc_library(
    name = "hood_plants",
    srcs = [
        "hood_plant.cc",
        "integral_hood_plant.cc",
    ],
    hdrs = [
        "hood_plant.h",
        "integral_hood_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "hood_encoder_zeroing_estimator",
    srcs = [
        "hood_encoder_zeroing_estimator.cc",
    ],
    hdrs = [
        "hood_encoder_zeroing_estimator.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/zeroing",
        "//frc971/zeroing:absolute_and_absolute_encoder",
        "//y2020:constants",
    ],
)
