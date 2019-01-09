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
        ":sensor_unwrap",
        "//aos:math",
        "//aos/controls:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/zeroing",
        "//y2018:constants",
        "//y2018/control_loops/superstructure:superstructure_output_fbs",
        "//y2018/control_loops/superstructure:superstructure_position_fbs",
        "//y2018/control_loops/superstructure:superstructure_status_fbs",
    ],
)

cc_library(
    name = "sensor_unwrap",
    srcs = [
        "sensor_unwrap.cc",
    ],
    hdrs = [
        "sensor_unwrap.h",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "unwrap_test",
    srcs = [
        "unwrap_test.cc",
    ],
    deps = [
        ":sensor_unwrap",
        "//aos/testing:googletest",
    ],
)
