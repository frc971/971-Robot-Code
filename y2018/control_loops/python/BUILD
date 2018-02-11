package(default_visibility = ["//y2018:__subpackages__"])

py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    restricted_to = ["//tools:k8"],
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
    restricted_to = ["//tools:k8"],
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
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:drivetrain",
    ],
)

py_test(
    name = "polydrivetrain_test",
    srcs = [
        "polydrivetrain_test.py",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":polydrivetrain_lib",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:polydrivetrain",
    ],
)

py_binary(
    name = "arm_trajectory",
    srcs = [
        "arm_trajectory.py",
        "path_points.py",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
    ],
)

py_binary(
    name = "intake",
    srcs = [
        "intake.py",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
    ],
)

cc_binary(
    name = "arm_mpc",
    srcs = [
        "arm_mpc.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        "//third_party/ct",
        "//third_party/gflags",
        "//third_party/matplotlib-cpp",
    ],
)
