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

cc_test(
    name = "catapult_test",
    srcs = [
        "catapult_test.cc",
    ],
    deps = [
        ":catapult_plants",
        "//aos/testing:googletest",
        "//frc971/control_loops/catapult",
    ],
)

cc_binary(
    name = "catapult_main",
    srcs = [
        "catapult_main.cc",
    ],
    deps = [
        ":catapult_plants",
        "//aos:init",
        "//frc971/control_loops/catapult",
    ],
)
