package(default_visibility = ["//y2014:__subpackages__"])

py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:drivetrain",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "polydrivetrain",
    srcs = [
        "drivetrain.py",
        "polydrivetrain.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:polydrivetrain",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_library(
    name = "polydrivetrain_lib",
    srcs = [
        "drivetrain.py",
        "polydrivetrain.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:drivetrain",
        "//frc971/control_loops/python:polydrivetrain",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "claw",
    srcs = [
        "claw.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":polydrivetrain_lib",
        ":python_init",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//matplotlib",
        "@pip//pygobject",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "shooter",
    srcs = [
        "shooter.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//matplotlib",
        "@pip//pygobject",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "extended_lqr",
    srcs = [
        "extended_lqr.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//matplotlib",
        "@pip//pygobject",
        "@pip//python_gflags",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//y2014/control_loops:python_init"],
)
