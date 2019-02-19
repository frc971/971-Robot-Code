package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")
load("//tools:environments.bzl", "mcu_cpus")
load("//tools/build_rules:select.bzl", "compiler_select", "cpu_select")

flatbuffer_cc_library(
    name = "drivetrain_goal_fbs",
    srcs = ["drivetrain_goal.fbs"],
    compatible_with = mcu_cpus,
    gen_reflections = 1,
    includes = ["//frc971/control_loops:control_loops_fbs_includes"],
)

flatbuffer_cc_library(
    name = "drivetrain_output_fbs",
    srcs = ["drivetrain_output.fbs"],
    compatible_with = mcu_cpus,
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "drivetrain_position_fbs",
    srcs = ["drivetrain_position.fbs"],
    compatible_with = mcu_cpus,
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "drivetrain_status_fbs",
    srcs = ["drivetrain_status.fbs"],
    compatible_with = mcu_cpus,
    gen_reflections = 1,
    includes = ["//frc971/control_loops:control_loops_fbs_includes"],
)

genrule(
    name = "drivetrain_goal_float_fbs_generated",
    srcs = ["drivetrain_goal.fbs"],
    outs = ["drivetrain_goal_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
    compatible_with = mcu_cpus,
)

genrule(
    name = "drivetrain_position_float_fbs_generated",
    srcs = ["drivetrain_position.fbs"],
    outs = ["drivetrain_position_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
    compatible_with = mcu_cpus,
)

genrule(
    name = "drivetrain_output_float_fbs_generated",
    srcs = ["drivetrain_output.fbs"],
    outs = ["drivetrain_output_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
    compatible_with = mcu_cpus,
)

genrule(
    name = "drivetrain_status_float_fbs_generated",
    srcs = ["drivetrain_status.fbs"],
    outs = ["drivetrain_status_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
    compatible_with = mcu_cpus,
)

flatbuffer_cc_library(
    name = "drivetrain_goal_float_fbs",
    srcs = ["drivetrain_goal_float.fbs"],
    compatible_with = mcu_cpus,
    gen_reflections = 1,
    includes = ["//frc971/control_loops:control_loops_fbs_includes"],
)

flatbuffer_cc_library(
    name = "drivetrain_output_float_fbs",
    srcs = ["drivetrain_output_float.fbs"],
    compatible_with = mcu_cpus,
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "drivetrain_position_float_fbs",
    srcs = ["drivetrain_position_float.fbs"],
    compatible_with = mcu_cpus,
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "drivetrain_status_float_fbs",
    srcs = ["drivetrain_status_float.fbs"],
    compatible_with = mcu_cpus,
    gen_reflections = 1,
    includes = ["//frc971/control_loops:control_loops_fbs_includes"],
)

aos_config(
    name = "config",
    src = "drivetrain_config.json",
    flatbuffers = [
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":drivetrain_position_fbs",
        ":localizer_fbs",
        "//frc971/queues:gyro",
        "//frc971/wpilib:imu_fbs",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/robot_state:config",
    ],
)

cc_library(
    name = "drivetrain_config",
    hdrs = [
        "drivetrain_config.h",
    ],
    deps = [
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "hybrid_ekf",
    hdrs = ["hybrid_ekf.h"],
    deps = [
        ":drivetrain_config",
        "//aos/containers:priority_queue",
        "//aos/util:math",
        "//frc971/control_loops:c2d",
        "//frc971/control_loops:runge_kutta",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "hybrid_ekf_test",
    srcs = ["hybrid_ekf_test.cc"],
    deps = [
        ":drivetrain_test_lib",
        ":hybrid_ekf",
        ":trajectory",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "//aos/testing:test_shm",
    ],
)

flatbuffer_cc_library(
    name = "localizer_fbs",
    srcs = ["localizer.fbs"],
    gen_reflections = 1,
)

cc_library(
    name = "localizer",
    hdrs = ["localizer.h"],
    deps = [
        ":drivetrain_config",
        ":hybrid_ekf",
        "//frc971/control_loops:pose",
    ],
)

cc_library(
    name = "gear",
    hdrs = [
        "gear.h",
    ],
    compatible_with = mcu_cpus,
)

cc_library(
    name = "splinedrivetrain",
    srcs = [
        "splinedrivetrain.cc",
    ],
    hdrs = [
        "splinedrivetrain.h",
    ],
    deps = [
        ":distance_spline",
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":spline",
        ":trajectory",
        "//aos:init",
        "//aos/util:math",
        "//frc971/control_loops:control_loops_fbs",
    ],
)

cc_library(
    name = "line_follow_drivetrain",
    srcs = [
        "line_follow_drivetrain.cc",
    ],
    hdrs = [
        "line_follow_drivetrain.h",
    ],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":localizer",
        "//aos:math",
        "//aos/util:math",
        "//frc971/control_loops:c2d",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:dlqr",
        "//frc971/control_loops:pose",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//y2019/control_loops/superstructure:superstructure_goal_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "line_follow_drivetrain_test",
    srcs = ["line_follow_drivetrain_test.cc"],
    linkstatic = True,
    restricted_to = ["//tools:k8"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_test_lib",
        ":line_follow_drivetrain",
        ":trajectory",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//third_party/matplotlib-cpp",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "ssdrivetrain",
    srcs = [
        "ssdrivetrain.cc",
    ],
    hdrs = [
        "ssdrivetrain.h",
    ],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":gear",
        ":localizer",
        "//aos:math",
        "//aos/controls:control_loop",
        "//aos/controls:polytope",
        "//aos/robot_state:robot_state_fbs",
        "//aos/util:log_interval",
        "//aos/util:trapezoid_profile",
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops:coerce_goal",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "polydrivetrain",
    srcs = [
        "polydrivetrain.cc",
    ],
    hdrs = [
        "polydrivetrain.h",
    ],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_status_fbs",
        ":gear",
        "//aos:math",
        "//aos/controls:polytope",
        "//aos/robot_state:robot_state_fbs",
        "//aos/util:log_interval",
        "//frc971/control_loops:coerce_goal",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "drivetrain_config_uc",
    hdrs = [
        "drivetrain_config.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops:state_feedback_loop_uc",
    ],
)

cc_library(
    name = "polydrivetrain_uc",
    srcs = [
        "polydrivetrain.cc",
    ],
    hdrs = [
        "polydrivetrain.h",
    ],
    copts = ["-Wno-type-limits"],
    restricted_to = mcu_cpus,
    deps = [
        ":drivetrain_config_uc",
        ":drivetrain_goal_float_fbs",
        ":drivetrain_output_float_fbs",
        ":drivetrain_position_float_fbs",
        ":drivetrain_status_float_fbs",
        ":gear",
        "//aos:math",
        "//aos/controls:polytope_uc",
        "//frc971/control_loops:coerce_goal_uc",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:state_feedback_loop_uc",
    ],
)

genrule(
    name = "genrule_down_estimator",
    outs = [
        "down_estimator.h",
        "down_estimator.cc",
    ],
    cmd = "$(location //frc971/control_loops/python:down_estimator) $(OUTS)",
    tools = [
        "//frc971/control_loops/python:down_estimator",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "down_estimator",
    srcs = [
        "down_estimator.cc",
    ],
    hdrs = [
        "down_estimator.h",
    ],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "drivetrain_lib",
    srcs = [
        "drivetrain.cc",
    ],
    hdrs = [
        "drivetrain.h",
    ],
    deps = [
        ":down_estimator",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_status_fbs",
        ":gear",
        ":line_follow_drivetrain",
        ":localizer",
        ":localizer_fbs",
        ":polydrivetrain",
        ":splinedrivetrain",
        ":ssdrivetrain",
        "//aos/controls:control_loop",
        "//aos/util:log_interval",
        "//frc971/control_loops:runge_kutta",
        "//frc971/queues:gyro",
        "//frc971/wpilib:imu_fbs",
    ],
)

cc_library(
    name = "drivetrain_test_lib",
    testonly = True,
    srcs = ["drivetrain_test_lib.cc"],
    hdrs = ["drivetrain_test_lib.h"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_status_fbs",
        ":trajectory",
        "//aos/events:event_loop",
        "//aos/testing:googletest",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/queues:gyro",
        "//y2016:constants",
        "//y2016/control_loops/drivetrain:polydrivetrain_plants",
    ],
)

cc_test(
    name = "drivetrain_lib_test",
    srcs = [
        "drivetrain_lib_test.cc",
    ],
    data = ["config.json"],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    linkstatic = True,
    deps = [
        ":drivetrain_config",
        ":drivetrain_lib",
        ":localizer_fbs",
        ":drivetrain_goal_fbs",
        ":drivetrain_status_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_test_lib",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//frc971/queues:gyro",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)

genrule(
    name = "genrule_haptic_wheel",
    outs = [
        "haptic_wheel.h",
        "haptic_wheel.cc",
        "integral_haptic_wheel.h",
        "integral_haptic_wheel.cc",
        "haptic_trigger.h",
        "haptic_trigger.cc",
        "integral_haptic_trigger.h",
        "integral_haptic_trigger.cc",
    ],
    cmd = "$(location //frc971/control_loops/python:haptic_wheel) $(OUTS)",
    compatible_with = mcu_cpus,
    tools = [
        "//frc971/control_loops/python:haptic_wheel",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "haptic_input_uc",
    srcs = [
        "haptic_trigger.cc",
        "haptic_wheel.cc",
        "integral_haptic_trigger.cc",
        "integral_haptic_wheel.cc",
    ],
    hdrs = [
        "haptic_trigger.h",
        "haptic_wheel.h",
        "integral_haptic_trigger.h",
        "integral_haptic_wheel.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "//frc971/control_loops:state_feedback_loop_uc",
    ],
)

cc_library(
    name = "haptic_wheel",
    srcs = [
        "haptic_trigger.cc",
        "haptic_wheel.cc",
        "integral_haptic_trigger.cc",
        "integral_haptic_wheel.cc",
    ],
    hdrs = [
        "haptic_trigger.h",
        "haptic_wheel.h",
        "integral_haptic_trigger.h",
        "integral_haptic_wheel.h",
    ],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "spline",
    srcs = ["spline.cc"],
    hdrs = ["spline.h"],
    deps = [
        "//frc971/control_loops:binomial",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "spline.so",
    srcs = ["libspline.cc"],
    linkshared = True,
    deps = [
        ":distance_spline",
        ":spline",
        ":trajectory",
        "//aos/logging:implementations",
        "//aos/network:team_number",
        "//y2019/control_loops/drivetrain:drivetrain_base",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "spline_test",
    srcs = [
        "spline_test.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":spline",
        "//aos/testing:googletest",
        "//third_party/matplotlib-cpp",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "distance_spline",
    srcs = ["distance_spline.cc"],
    hdrs = ["distance_spline.h"],
    deps = [
        ":spline",
        "//aos/logging",
        "//frc971/control_loops:fixed_quadrature",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "distance_spline_test",
    srcs = [
        "distance_spline_test.cc",
    ],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    linkstatic = True,
    deps = [
        ":distance_spline",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "@com_github_gflags_gflags//:gflags",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)

cc_library(
    name = "trajectory",
    srcs = ["trajectory.cc"],
    hdrs = ["trajectory.h"],
    deps = [
        ":distance_spline",
        ":drivetrain_config",
        "//frc971/control_loops:c2d",
        "//frc971/control_loops:dlqr",
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:runge_kutta",
        "//frc971/control_loops:state_feedback_loop",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "trajectory_plot",
    srcs = [
        "trajectory_plot.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":distance_spline",
        ":trajectory",
        "//aos/logging:implementations",
        "//aos/network:team_number",
        "//third_party/matplotlib-cpp",
        "//y2019/control_loops/drivetrain:drivetrain_base",
        "@com_github_gflags_gflags//:gflags",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = [
        "trajectory_test.cc",
    ],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    linkstatic = True,
    deps = [
        ":trajectory",
        ":drivetrain_test_lib",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//y2016:constants",
        "//y2016/control_loops/drivetrain:polydrivetrain_plants",
        "//y2019/control_loops/drivetrain:drivetrain_base",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)
