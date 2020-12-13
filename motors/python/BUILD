py_binary(
    name = "big_phase_current",
    srcs = [
        "big_phase_current.py",
    ],
    data = [
        "@python_repo//:scipy",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

py_binary(
    name = "haptic_phase_current",
    srcs = [
        "haptic_phase_current.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
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
    deps = ["//motors:python_init"],
)
