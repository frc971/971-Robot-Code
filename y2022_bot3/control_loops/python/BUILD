package(default_visibility = ["//y2022_bot3:__subpackages__"])

py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
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
    target_compatible_with = ["@platforms//cpu:x86_64"],
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
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:drivetrain",
        "//frc971/control_loops/python:polydrivetrain",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "climber",
    srcs = [
        "climber.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:linear_system",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "intake",
    srcs = [
        "intake.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:angular_system",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//y2022_bot3/control_loops:python_init"],
)
