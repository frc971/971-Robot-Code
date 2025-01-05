package(default_visibility = ["//y2024_bot3:__subpackages__"])

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = ["//y2024_bot3/control_loops:python_init"],
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
        "//frc971/control_loops/python:angular_system",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "rotation",
    srcs = [
        "rotation.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//y2024_bot3:__subpackages__"],
    deps = [
        "//frc971/control_loops/python:angular_system_current",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)
