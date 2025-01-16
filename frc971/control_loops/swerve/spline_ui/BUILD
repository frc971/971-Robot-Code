py_binary(
    name = "server",
    srcs = [
        "server.py",
    ],
    data = [
        "//frc971/control_loops/swerve/spline_ui/www:static_files",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":trajectory_solver",
        "@pip//flask",
    ],
)

py_library(
    name = "trajectory_solver",
    srcs = [
        "trajectory_solver.py",
    ],
    deps = [
        "//frc971/control_loops/python:swerve_trajectory_optimizer",
        "@pip//casadi",
    ],
)

py_test(
    name = "trajectory_solver_test",
    srcs = [
        "trajectory_solver_test.py",
    ],
    deps = [
        ":trajectory_solver",
    ],
)
