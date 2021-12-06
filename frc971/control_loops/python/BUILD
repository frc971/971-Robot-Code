package(default_visibility = ["//visibility:public"])

py_binary(
    name = "haptic_wheel",
    srcs = [
        "haptic_wheel.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:controls",
        "@matplotlib_repo//:matplotlib3",
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
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//external:python-glog",
        "@python_repo//:scipy",
    ],
)

py_test(
    name = "polytope_test",
    srcs = [
        "polytope_test.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
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
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":controls",
        ":python_init",
        "@matplotlib_repo//:matplotlib3",
    ],
)

py_library(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":controls",
        ":python_init",
        "@matplotlib_repo//:matplotlib3",
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":python_init",
    ],
)

py_test(
    name = "lib_spline_test",
    srcs = [
        "lib_spline_test.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":libspline",
        ":python_init",
        "@python_repo//:numpy",
    ],
)

py_library(
    name = "polydrivetrain",
    srcs = [
        "polydrivetrain.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":controls",
        ":drivetrain",
        ":python_init",
        "//external:python-glog",
        "@matplotlib_repo//:matplotlib3",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//frc971/control_loops:python_init"],
)

py_library(
    name = "linear_system",
    srcs = ["linear_system.py"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        ":controls",
        "//aos/util:py_trapezoid_profile",
        "//frc971/control_loops:python_init",
        "@matplotlib_repo//:matplotlib3",
    ],
)

py_library(
    name = "angular_system",
    srcs = ["angular_system.py"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        ":controls",
        "//aos/util:py_trapezoid_profile",
        "//frc971/control_loops:python_init",
        "@matplotlib_repo//:matplotlib3",
    ],
)

py_binary(
    name = "spline_graph",
    srcs = [
        "color.py",
        "graph.py",
        "path_edit.py",
        "points.py",
        "spline_drawing.py",
        "spline_graph.py",
        "spline_writer.py",
    ],
    data = glob([
        "field_images/*.png",
        "field_images/*.svg",
    ]),
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        ":basic_window",
        ":libspline",
        ":python_init",
        "@matplotlib_repo//:matplotlib3",
        "@python_gtk",
    ],
)

py_library(
    name = "basic_window",
    srcs = [
        "basic_window.py",
        "color.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
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
    target_compatible_with = ["@platforms//cpu:x86_64"],
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
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":controls",
        ":linear_system",
        ":python_init",
        "//external:python-gflags",
        "//external:python-glog",
    ],
)
