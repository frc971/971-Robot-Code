package(default_visibility = ["//y2025:__subpackages__"])

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = ["//y2025/control_loops:python_init"],
)

py_binary(
    name = "rotation",
    srcs = [
        "rotation.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//y2025:__subpackages__"],
    deps = [
        "//frc971/control_loops/python:angular_system_current",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "elevator",
    srcs = [
        "elevator.py",
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
    name = "ground_intake_pivot",
    srcs = [
        "ground_intake_pivot.py",
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

py_binary(
    name = "pivot",
    srcs = [
        "pivot.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//y2025:__subpackages__"],
    deps = [
        "//frc971/control_loops/python:angular_system",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
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
        "//frc971/control_loops/python:angular_system",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)
