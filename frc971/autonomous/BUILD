load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "auto_fbs",
    srcs = ["auto.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "auto_mode_fbs",
    srcs = ["auto_mode.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "base_autonomous_actor",
    srcs = [
        "base_autonomous_actor.cc",
    ],
    hdrs = [
        "base_autonomous_actor.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":auto_fbs",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/util:math",
        "//aos/util:phased_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//frc971/control_loops/drivetrain:drivetrain_goal_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/control_loops/drivetrain:localizer_fbs",
        "//frc971/control_loops/drivetrain:spline",
        "//frc971/control_loops/drivetrain:spline_goal_fbs",
        "//y2019/control_loops/drivetrain:target_selector_fbs",
    ],
)

cc_library(
    name = "user_button_localized_autonomous_actor",
    srcs = [
        "user_button_localized_autonomous_actor.cc",
    ],
    hdrs = [
        "user_button_localized_autonomous_actor.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":base_autonomous_actor",
    ],
)

aos_config(
    name = "aos_config",
    src = "autonomous_config.json",
    flatbuffers = [
        "//aos/actions:actions_fbs",
        ":auto_fbs",
        ":auto_mode_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
