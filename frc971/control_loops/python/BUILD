package(default_visibility = ["//visibility:public"])

py_binary(
    name = "haptic_wheel",
    srcs = [
        "haptic_wheel.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

py_library(
    name = "controls",
    srcs = [
        "cim.py",
        "control_loop.py",
        "controls.py",
        "libcdd.py",
        "polytope.py",
    ],
    data = [
        "//third_party/cddlib:_cddlib.so",
        "@python_repo//:scipy",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":python_init",
        "//external:python-glog",
        "@slycot_repo//:slycot",
    ],
)

py_test(
    name = "polytope_test",
    srcs = [
        "polytope_test.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        ":controls",
        ":python_init",
    ],
)

py_binary(
    name = "down_estimator",
    srcs = [
        "down_estimator.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        ":controls",
        ":python_init",
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

py_library(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":controls",
        ":python_init",
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

py_library(
    name = "libspline",
    srcs = [
        "libspline.py",
    ],
    data = [
        "//frc971/control_loops/drivetrain:spline.so",
    ],
    deps = [
        ":python_init",
    ],
)

py_test(
    name = "lib_spline_test",
    srcs = [
        "lib_spline_test.py",
    ],
    deps = [
        ":libspline",
        ":python_init",
    ],
)

py_library(
    name = "polydrivetrain",
    srcs = [
        "polydrivetrain.py",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":controls",
        ":drivetrain",
        ":python_init",
        "//external:python-glog",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = ["//frc971/control_loops:python_init"],
)

py_binary(
    name = "spline",
    srcs = [
        "spline.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "//y2016/control_loops/python:polydrivetrain_lib",
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

py_library(
    name = "linear_system",
    srcs = ["linear_system.py"],
    restricted_to = ["//tools:k8"],
    visibility = ["//visibility:public"],
    deps = [
        ":controls",
        "//aos/util:py_trapezoid_profile",
        "//frc971/control_loops:python_init",
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

py_library(
    name = "angular_system",
    srcs = ["angular_system.py"],
    restricted_to = ["//tools:k8"],
    visibility = ["//visibility:public"],
    deps = [
        ":controls",
        "//aos/util:py_trapezoid_profile",
        "//frc971/control_loops:python_init",
        "@matplotlib_repo//:matplotlib2.7",
    ],
)

py_binary(
    name = "path_edit",
    srcs = [
        "basic_window.py",
        "color.py",
        "path_edit.py",
    ],
    restricted_to = ["//tools:k8"],
    visibility = ["//visibility:public"],
    deps = [
        ":libspline",
        ":python_init",
        "@python_gtk",
    ],
)

py_library(
    name = "basic_window",
    srcs = [
        "basic_window.py",
        "color.py",
    ],
    restricted_to = ["//tools:k8"],
    visibility = ["//visibility:public"],
    deps = [
        ":python_init",
        "@python_gtk",
    ],
)

py_library(
    name = "color",
    srcs = [
        "color.py",
    ],
    restricted_to = ["//tools:k8"],
    visibility = ["//visibility:public"],
    deps = [
        ":python_init",
    ],
)

py_binary(
    name = "static_zeroing_single_dof_profiled_subsystem_test",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        ":controls",
        ":linear_system",
        ":python_init",
        "//external:python-gflags",
        "//external:python-glog",
    ],
)
