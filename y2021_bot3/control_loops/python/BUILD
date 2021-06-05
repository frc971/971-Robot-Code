package(default_visibility = ["//y2021_bot3:__subpackages__"])

py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:drivetrain",
    ],
)

py_binary(
    name = "polydrivetrain",
    srcs = [
        "drivetrain.py",
        "polydrivetrain.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:polydrivetrain",
    ],
)

py_library(
    name = "polydrivetrain_lib",
    srcs = [
        "drivetrain.py",
        "polydrivetrain.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:drivetrain",
        "//frc971/control_loops/python:polydrivetrain",
    ],
)
