package(default_visibility = ["//y2024:__subpackages__"])

genrule(
    name = "genrule_catapult",
    outs = [
        "catapult_plant.h",
        "catapult_plant.cc",
        "catapult_plant.json",
        "integral_catapult_plant.h",
        "integral_catapult_plant.cc",
        "integral_catapult_plant.json",
    ],
    cmd = "$(location //y2024/control_loops/python:catapult) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2024/control_loops/python:catapult",
    ],
)

cc_library(
    name = "catapult_plants",
    srcs = [
        "catapult_plant.cc",
        "integral_catapult_plant.cc",
    ],
    hdrs = [
        "catapult_plant.h",
        "integral_catapult_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

filegroup(
    name = "catapult_json",
    srcs = ["integral_catapult_plant.json"],
    visibility = ["//visibility:public"],
)
