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
