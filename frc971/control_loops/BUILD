package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")
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
    name = "hall_effect_tracker",
    hdrs = [
        "hall_effect_tracker.h",
    ],
    deps = [
        ":queues",
    ],
)

queue_library(
    name = "queues",
    srcs = [
        "control_loops.q",
    ],
)

cc_test(
    name = "position_sensor_sim_test",
    srcs = [
        "position_sensor_sim_test.cc",
    ],
    deps = [
        ":position_sensor_sim",
        ":queues",
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
        ":gaussian_noise",
        ":queues",
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
        "//third_party/eigen",
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
        "//third_party/eigen",
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
        "//third_party/eigen",
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
        "//third_party/eigen",
    ],
)

cc_library(
    name = "hybrid_state_feedback_loop",
    hdrs = [
        "hybrid_state_feedback_loop.h",
    ],
    deps = [
        ":state_feedback_loop",
        "//aos:macros",
        "//aos/controls:control_loop",
        "//aos/logging",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "simple_capped_state_feedback_loop",
    hdrs = [
        "simple_capped_state_feedback_loop.h",
    ],
    deps = [
        ":state_feedback_loop",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "runge_kutta",
    hdrs = [
        "runge_kutta.h",
    ],
    deps = [
        "//third_party/eigen",
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
        "//third_party/eigen",
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

queue_library(
    name = "profiled_subsystem_queue",
    srcs = [
        "profiled_subsystem.q",
    ],
    deps = [
        ":queues",
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
        ":profiled_subsystem_queue",
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
        "//third_party/eigen",
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
        "//third_party/eigen",
    ],
)

cc_library(
    name = "dlqr",
    hdrs = [
        "dlqr.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/eigen",
        "@slycot_repo//:slicot",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = ["//frc971:python_init"],
)
