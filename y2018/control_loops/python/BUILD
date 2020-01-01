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
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

cc_library(
    name = "dlqr",
    hdrs = [
        "dlqr.h",
    ],
    deps = [
        "@slycot_repo//:slicot",
    ],
)

cc_binary(
    name = "arm_mpc",
    srcs = [
        "arm_mpc.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":arm_bounds",
        ":dlqr",
        "//third_party/ct",
        "//third_party/matplotlib-cpp",
        "@cgal_repo//:cgal",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "3d_plot",
    srcs = [
        "3d_plot.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":arm_bounds",
        "//third_party/matplotlib-cpp",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "arm_bounds",
    srcs = [
        "arm_bounds.cc",
    ],
    hdrs = [
        "arm_bounds.h",
    ],
    deps = [
        "@cgal_repo//:cgal",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "2d_plot",
    srcs = [
        "2d_plot.cc",
    ],
    restricted_to = ["//tools:k8"],
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
    default_python_version = "PY3",
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    srcs_version = "PY3",
    deps = [
        ":python_init",
        "//frc971/control_loops/python:basic_window",
        "//frc971/control_loops/python:color",
        "@python_gtk",
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

py_binary(
    name = "extended_lqr",
    srcs = [
        "extended_lqr.py",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
    ],
)

py_binary(
    name = "arm_mpc_py",
    srcs = [
        "arm_mpc.py",
    ],
    main = "arm_mpc.py",
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
    ],
)
