py_binary(
    name = "rotation",
    srcs = [
        "rotation.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//y2024_swerve:__subpackages__"],
    deps = [
        "//frc971/control_loops/python:angular_system_current",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)
