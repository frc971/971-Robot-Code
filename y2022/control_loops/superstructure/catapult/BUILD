genrule(
    name = "genrule_catapult",
    outs = [
        "catapult_plant.h",
        "catapult_plant.cc",
        "integral_catapult_plant.h",
        "integral_catapult_plant.cc",
    ],
    cmd = "$(location //y2022/control_loops/python:catapult) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2022/control_loops/python:catapult",
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
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "catapult",
    srcs = [
        "catapult.cc",
    ],
    hdrs = [
        "catapult.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":catapult_plants",
        "//aos:realtime",
        "//frc971/zeroing:pot_and_absolute_encoder",
        "//third_party/osqp-cpp",
        "//y2022:constants",
        "//y2022/control_loops/superstructure:superstructure_goal_fbs",
        "//y2022/control_loops/superstructure:superstructure_position_fbs",
        "//y2022/control_loops/superstructure:superstructure_status_fbs",
    ],
)

cc_test(
    name = "catapult_test",
    srcs = [
        "catapult_test.cc",
    ],
    deps = [
        ":catapult",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "catapult_main",
    srcs = [
        "catapult_main.cc",
    ],
    deps = [
        ":catapult",
        "//aos:init",
    ],
)
