genrule(
    name = "genrule_intake",
    outs = [
        "intake_plant.h",
        "intake_plant.cc",
        "intake_delayed_plant.h",
        "intake_delayed_plant.cc",
    ],
    cmd = "$(location //y2018/control_loops/python:intake) $(OUTS)",
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
    visibility = ["//visibility:public"],
    deps = [
        ":intake_plants",
        "//aos/common:math",
        "//aos/common/controls:control_loop",
        "//aos/common/logging:queue_logging",
        "//frc971/control_loops:queues",
        "//frc971/zeroing",
        "//y2018:constants",
        "//y2018/control_loops/superstructure:superstructure_queue",
    ],
)
