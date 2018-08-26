package(default_visibility = ["//y2018:__subpackages__"])

py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
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
    restricted_to = ["//tools:k8"],
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
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:drivetrain",
    ],
)

py_test(
    name = "polydrivetrain_test",
    srcs = [
        "polydrivetrain_test.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        ":polydrivetrain_lib",
        ":python_init",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:polydrivetrain",
    ],
)

py_binary(
    name = "arm_trajectory",
    srcs = [
        "arm_trajectory.py",
        "path_points.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        ":python_init",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
    ],
)

py_binary(
    name = "intake",
    srcs = [
        "intake.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        ":python_init",
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "@matplotlib",
    ],
)

cc_binary(
    name = "arm_mpc",
    srcs = [
        "arm_mpc.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        "//third_party/ct",
        "//third_party/gflags",
        "//third_party/matplotlib-cpp",
    ],
)

py_binary(
    name = "graph_edit",
    srcs = [
        "basic_window.py",
        "graph_edit.py",
        "graph_generate.py",
    ],
    default_python_version = "PY3",
    legacy_create_init = False,
    srcs_version = "PY3",
    deps = [
        ":python_init",
    ],
)

py_binary(
    name = "graph_codegen",
    srcs = [
        "graph_codegen.py",
        "graph_generate.py",
    ],
    default_python_version = "PY2",
    legacy_create_init = False,
    srcs_version = "PY2",
    deps = [
        ":python_init",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = ["//y2018/control_loops:python_init"],
)
