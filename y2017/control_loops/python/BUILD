package(default_visibility = ["//y2017:__subpackages__"])

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
    deps = [
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:drivetrain",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "shooter",
    srcs = [
        "shooter.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//matplotlib",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "indexer",
    srcs = [
        "indexer.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:controls",
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
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:linear_system",
        "@pip//glog",
        "@pip//python_gflags",
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
        "//aos/util:py_trapezoid_profile",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//matplotlib",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "hood",
    srcs = [
        "hood.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//aos/util:py_trapezoid_profile",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//matplotlib",
        "@pip//python_gflags",
    ],
)

py_library(
    name = "turret_lib",
    srcs = [
        "turret.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//aos/util:py_trapezoid_profile",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_library(
    name = "indexer_lib",
    srcs = [
        "indexer.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "column",
    srcs = [
        "column.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":indexer_lib",
        ":python_init",
        ":turret_lib",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//matplotlib",
        "@pip//python_gflags",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//y2017/control_loops:python_init"],
)
