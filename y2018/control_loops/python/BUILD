package(default_visibility = ["//y2018:__subpackages__"])

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
        ":python_init",
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:drivetrain",
        "//frc971/control_loops/python:polydrivetrain",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_test(
    name = "polydrivetrain_test",
    srcs = [
        "polydrivetrain_test.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":polydrivetrain_lib",
        ":python_init",
        "//frc971/control_loops/python:controls",
        "//frc971/control_loops/python:polydrivetrain",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "arm_trajectory",
    srcs = [
        "arm_trajectory.py",
        "path_points.py",
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
        "@pip//glog",
        "@pip//matplotlib",
        "@pip//pygobject",
        "@pip//python_gflags",
    ],
)

cc_binary(
    name = "2d_plot",
    srcs = [
        "2d_plot.cc",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//third_party/matplotlib-cpp",
        "@com_github_gflags_gflags//:gflags",
    ],
)

py_binary(
    name = "graph_edit",
    srcs = [
        "graph_edit.py",
        "graph_generate.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:basic_window",
        "//frc971/control_loops/python:color",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//shapely",
    ],
)

py_binary(
    name = "graph_codegen",
    srcs = [
        "graph_codegen.py",
        "graph_generate.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":python_init",
        "@pip//numpy",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//y2018/control_loops:python_init"],
)

py_binary(
    name = "extended_lqr",
    srcs = [
        "extended_lqr.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)
