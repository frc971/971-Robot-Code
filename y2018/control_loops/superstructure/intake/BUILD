genrule(
    name = "genrule_intake",
    outs = [
        "intake_plant.h",
        "intake_plant.cc",
        "intake_delayed_plant.h",
        "intake_delayed_plant.cc",
    ],
    cmd = "$(location //y2018/control_loops/python:intake) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2018/control_loops/python:intake",
    ],
)

cc_library(
    name = "intake_plants",
    srcs = [
        "intake_delayed_plant.cc",
        "intake_plant.cc",
    ],
    hdrs = [
        "intake_delayed_plant.h",
        "intake_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "intake",
    srcs = [
        "intake.cc",
    ],
    hdrs = [
        "intake.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":intake_plants",
        "//aos:math",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/zeroing",
        "//frc971/zeroing:pot_and_absolute_encoder",
        "//y2018:constants",
        "//y2018/control_loops/superstructure:superstructure_output_fbs",
        "//y2018/control_loops/superstructure:superstructure_position_fbs",
        "//y2018/control_loops/superstructure:superstructure_status_fbs",
    ],
)
