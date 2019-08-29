package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "team_number_test_environment",
    testonly = True,
    srcs = [
        "team_number_test_environment.cc",
    ],
    hdrs = [
        "team_number_test_environment.h",
    ],
    deps = [
        "//aos/network:team_number",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "hybrid_state_feedback_loop_test",
    srcs = [
        "hybrid_state_feedback_loop_test.cc",
    ],
    deps = [
        ":hybrid_state_feedback_loop",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "pose",
    hdrs = ["pose.h"],
    deps = [
        "//aos/util:math",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "pose_test",
    srcs = ["pose_test.cc"],
    deps = [
        ":pose",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "hall_effect_tracker",
    hdrs = [
        "hall_effect_tracker.h",
    ],
    deps = [
        ":control_loops_fbs",
    ],
)

flatbuffer_cc_library(
    name = "control_loops_fbs",
    srcs = [
        "control_loops.fbs",
    ],
    compatible_with = mcu_cpus,
)

cc_test(
    name = "position_sensor_sim_test",
    srcs = [
        "position_sensor_sim_test.cc",
    ],
    deps = [
        ":control_loops_fbs",
        ":position_sensor_sim",
        "//aos/logging",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "position_sensor_sim",
    testonly = True,
    srcs = [
        "position_sensor_sim.cc",
    ],
    hdrs = [
        "position_sensor_sim.h",
    ],
    linkopts = [
        "-lm",
    ],
    deps = [
        ":control_loops_fbs",
        ":gaussian_noise",
        "//aos/testing:random_seed",
    ],
)

cc_library(
    name = "gaussian_noise",
    srcs = [
        "gaussian_noise.cc",
    ],
    hdrs = [
        "gaussian_noise.h",
    ],
    linkopts = [
        "-lm",
    ],
)

cc_library(
    name = "coerce_goal_uc",
    srcs = [
        "coerce_goal.cc",
    ],
    hdrs = [
        "coerce_goal.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "//aos/controls:polytope_uc",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "coerce_goal",
    srcs = [
        "coerce_goal.cc",
    ],
    hdrs = [
        "coerce_goal.h",
    ],
    linkopts = [
        "-lm",
    ],
    deps = [
        "//aos/controls:polytope",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

# TODO(austin): Select isn't working right.  We should be able to remove
# logging conditionally with select and have CPU constraints work correctly.
cc_library(
    name = "state_feedback_loop_uc",
    hdrs = [
        "state_feedback_loop.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "//aos:macros",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "state_feedback_loop",
    hdrs = [
        "state_feedback_loop.h",
    ],
    deps = [
        "//aos:macros",
        "//aos/logging",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "hybrid_state_feedback_loop",
    hdrs = [
        "hybrid_state_feedback_loop.h",
    ],
    deps = [
        ":c2d",
        ":state_feedback_loop",
        "//aos:macros",
        "//aos/controls:control_loop",
        "//aos/logging",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "simple_capped_state_feedback_loop",
    hdrs = [
        "simple_capped_state_feedback_loop.h",
    ],
    deps = [
        ":state_feedback_loop",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "runge_kutta",
    hdrs = [
        "runge_kutta.h",
    ],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "runge_kutta_test",
    srcs = [
        "runge_kutta_test.cc",
    ],
    deps = [
        ":runge_kutta",
        "//aos/testing:googletest",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "fixed_quadrature",
    hdrs = [
        "fixed_quadrature.h",
    ],
)

cc_test(
    name = "fixed_quadrature_test",
    srcs = [
        "fixed_quadrature_test.cc",
    ],
    deps = [
        ":fixed_quadrature",
        "//aos/testing:googletest",
    ],
)

flatbuffer_cc_library(
    name = "profiled_subsystem_fbs",
    srcs = [
        "profiled_subsystem.fbs",
    ],
    includes = [
        ":control_loops_fbs_includes",
    ],
)

cc_library(
    name = "profiled_subsystem",
    srcs = [
        "profiled_subsystem.cc",
    ],
    hdrs = [
        "profiled_subsystem.h",
    ],
    deps = [
        ":control_loops_fbs",
        ":profiled_subsystem_fbs",
        ":simple_capped_state_feedback_loop",
        ":state_feedback_loop",
        "//aos/controls:control_loop",
        "//aos/util:trapezoid_profile",
        "//frc971/zeroing",
    ],
)

cc_library(
    name = "jacobian",
    hdrs = [
        "jacobian.h",
    ],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "jacobian_test",
    srcs = [
        "jacobian_test.cc",
    ],
    deps = [
        ":jacobian",
        "//aos/testing:googletest",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "c2d_test",
    srcs = [
        "c2d_test.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":c2d",
        ":runge_kutta",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "c2d",
    hdrs = [
        "c2d.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/time",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "dlqr",
    hdrs = [
        "dlqr.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
        "@slycot_repo//:slicot",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = ["//frc971:python_init"],
)

cc_library(
    name = "binomial",
    hdrs = ["binomial.h"],
)

cc_test(
    name = "binomial_test",
    srcs = [
        "binomial_test.cc",
    ],
    deps = [
        ":binomial",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "capped_test_plant",
    testonly = True,
    srcs = [
        "capped_test_plant.cc",
    ],
    hdrs = [
        "capped_test_plant.h",
    ],
    deps = [
        ":state_feedback_loop",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "static_zeroing_single_dof_profiled_subsystem",
    hdrs = [
        "static_zeroing_single_dof_profiled_subsystem.h",
    ],
    deps = [
        "//frc971/control_loops:profiled_subsystem",
    ],
)

genrule(
    name = "genrule_static_zeroing_single_dof_profiled_subsystem_test",
    outs = [
        "static_zeroing_single_dof_profiled_subsystem_test_plant.h",
        "static_zeroing_single_dof_profiled_subsystem_test_plant.cc",
        "static_zeroing_single_dof_profiled_subsystem_test_integral_plant.h",
        "static_zeroing_single_dof_profiled_subsystem_test_integral_plant.cc",
    ],
    cmd = "$(location //frc971/control_loops/python:static_zeroing_single_dof_profiled_subsystem_test) $(OUTS)",
    tools = [
        "//frc971/control_loops/python:static_zeroing_single_dof_profiled_subsystem_test",
    ],
)

cc_library(
    name = "static_zeroing_single_dof_profiled_subsystem_test_plants",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test_integral_plant.cc",
        "static_zeroing_single_dof_profiled_subsystem_test_plant.cc",
    ],
    hdrs = [
        "static_zeroing_single_dof_profiled_subsystem_test_integral_plant.h",
        "static_zeroing_single_dof_profiled_subsystem_test_plant.h",
    ],
    deps = [
        ":state_feedback_loop",
    ],
)

flatbuffer_cc_library(
    name = "static_zeroing_single_dof_profiled_subsystem_test_fbs",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test.fbs",
    ],
    includes = [
        ":control_loops_fbs_includes",
        ":profiled_subsystem_fbs_includes",
    ],
)

cc_test(
    name = "static_zeroing_single_dof_profiled_subsystem_test",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test.cc",
    ],
    deps = [
        ":capped_test_plant",
        ":position_sensor_sim",
        ":static_zeroing_single_dof_profiled_subsystem",
        ":static_zeroing_single_dof_profiled_subsystem_test_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_plants",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
    ],
)
