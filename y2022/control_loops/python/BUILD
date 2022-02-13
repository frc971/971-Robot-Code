package(default_visibility = ["//y2022:__subpackages__"])

py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
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
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
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

py_binary(
    name = "intake",
    srcs = [
        "intake.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:angular_system",
        "//frc971/control_loops/python:controls",
    ],
)

py_binary(
    name = "turret",
    srcs = [
        "turret.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:angular_system",
        "//frc971/control_loops/python:controls",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//y2022/control_loops:python_init"],
)
