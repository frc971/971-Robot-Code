package(default_visibility = ["//visibility:public"])

load("//aos:config.bzl", "aos_config")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "robot_state_fbs",
    srcs = ["robot_state.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "joystick_state_fbs",
    srcs = ["joystick_state.fbs"],
    gen_reflections = 1,
)

aos_config(
    name = "config",
    src = "robot_state_config.json",
    flatbuffers = [
        "//aos/robot_state:joystick_state_fbs",
        "//aos/robot_state:robot_state_fbs",
    ],
    visibility = ["//visibility:public"],
    deps = ["//aos/events:config"],
)
