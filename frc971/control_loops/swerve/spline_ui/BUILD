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

py_binary(
    name = "generate_solved_trajectory",
    srcs = [
        "generate_solved_trajectory.py",
    ],
    data = [
        ":testing_swerve_path",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":trajectory_solver",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

filegroup(
    name = "testing_swerve_path",
    srcs = [
        "testing_swerve_path.json",
    ],
)
