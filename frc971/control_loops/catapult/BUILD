load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "catapult_goal_fbs",
    srcs = [
        "catapult_goal.fbs",
    ],
    deps = [
        "//frc971/control_loops:profiled_subsystem_fbs",
    ],
)

cc_library(
    name = "catapult_controller",
    srcs =
        [
            "catapult_controller.cc",
        ],
    hdrs =
        [
            "catapult_controller.h",
        ],
    visibility = ["//visibility:public"],
    deps =
        [
            ":mpc_problem_generator",
        ],
)

cc_library(
    name = "mpc_problem",
    srcs =
        [
            "mpc_problem.cc",
        ],
    hdrs =
        [
            "mpc_problem.h",
        ],
    visibility = ["//visibility:public"],
    deps =
        [
            "//aos:realtime",
            "//aos/time",
            "//third_party/osqp-cpp",
            "@org_tuxfamily_eigen//:eigen",
        ],
)

cc_library(
    name = "mpc_problem_generator",
    srcs =
        [
            "mpc_problem_generator.cc",
        ],
    hdrs =
        [
            "mpc_problem_generator.h",
        ],
    visibility = ["//visibility:public"],
    deps =
        [
            ":mpc_problem",
            "//frc971/control_loops:state_feedback_loop",
        ],
)

cc_library(
    name = "catapult",
    srcs = [
        "catapult.cc",
    ],
    hdrs =
        [
            "catapult.h",
        ],
    visibility = ["//visibility:public"],
    deps = [
        ":catapult_controller",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//frc971/control_loops/catapult:catapult_goal_fbs",
        "//frc971/zeroing:pot_and_absolute_encoder",
    ],
)
