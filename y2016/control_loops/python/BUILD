package(default_visibility = ["//y2016:__subpackages__"])

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
        ":python_init",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:drivetrain",
        "//frc971/control_loops/python:polydrivetrain",
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
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "@matplotlib_repo//:matplotlib3",
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
    name = "shoulder",
    srcs = [
        "shoulder.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//aos/util:py_trapezoid_profile",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "@matplotlib_repo//:matplotlib3",
    ],
)

py_binary(
    name = "wrist",
    srcs = [
        "wrist.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//aos/util:py_trapezoid_profile",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "@matplotlib_repo//:matplotlib3",
    ],
)

py_library(
    name = "wrist_lib",
    srcs = [
        "wrist.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//aos/util:py_trapezoid_profile",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
    ],
)

py_library(
    name = "shoulder_lib",
    srcs = [
        "shoulder.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//aos/util:py_trapezoid_profile",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "@matplotlib_repo//:matplotlib3",
    ],
)

py_library(
    name = "arm_lib",
    srcs = [
        "arm.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":shoulder_lib",
        ":wrist_lib",
        "//aos/util:py_trapezoid_profile",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "@matplotlib_repo//:matplotlib3",
    ],
)

py_binary(
    name = "arm",
    srcs = [
        "arm.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        ":shoulder_lib",
        ":wrist_lib",
        "//aos/util:py_trapezoid_profile",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//y2016/control_loops:python_init"],
)
