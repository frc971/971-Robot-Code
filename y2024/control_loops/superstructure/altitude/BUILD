package(default_visibility = ["//y2024:__subpackages__"])

genrule(
    name = "genrule_altitude",
    outs = [
        "altitude_plant.h",
        "altitude_plant.cc",
        "altitude_plant.json",
        "integral_altitude_plant.h",
        "integral_altitude_plant.cc",
        "integral_altitude_plant.json",
    ],
    cmd = "$(location //y2024/control_loops/python:altitude) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2024/control_loops/python:altitude",
    ],
)

cc_library(
    name = "altitude_plants",
    srcs = [
        "altitude_plant.cc",
        "integral_altitude_plant.cc",
    ],
    hdrs = [
        "altitude_plant.h",
        "integral_altitude_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

filegroup(
    name = "altitude_json",
    srcs = ["integral_altitude_plant.json"],
    visibility = ["//visibility:public"],
)
