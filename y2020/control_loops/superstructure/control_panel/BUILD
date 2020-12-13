package(default_visibility = ["//y2020:__subpackages__"])

genrule(
    name = "genrule_control_panel",
    outs = [
        "control_panel_plant.h",
        "control_panel_plant.cc",
        "integral_control_panel_plant.h",
        "integral_control_panel_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:control_panel) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2020/control_loops/python:control_panel",
    ],
)

cc_library(
    name = "control_panel_plants",
    srcs = [
        "control_panel_plant.cc",
        "integral_control_panel_plant.cc",
    ],
    hdrs = [
        "control_panel_plant.h",
        "integral_control_panel_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)
