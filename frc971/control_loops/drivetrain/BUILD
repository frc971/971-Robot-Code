package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")
load("//tools/build_rules:select.bzl", "cpu_select")
load("@npm_bazel_typescript//:defs.bzl", "ts_library")

flatbuffer_cc_library(
    name = "spline_goal_fbs",
    srcs = ["spline_goal.fbs"],
    gen_reflections = 1,
    includes = ["//frc971/control_loops:control_loops_fbs_includes"],
)

flatbuffer_cc_library(
    name = "drivetrain_goal_fbs",
    srcs = ["drivetrain_goal.fbs"],
    gen_reflections = 1,
    includes = [
        ":spline_goal_fbs_includes",
        "//frc971/control_loops:control_loops_fbs_includes",
    ],
)

flatbuffer_cc_library(
    name = "drivetrain_output_fbs",
    srcs = ["drivetrain_output.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "drivetrain_position_fbs",
    srcs = ["drivetrain_position.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "drivetrain_status_fbs",
    srcs = ["drivetrain_status.fbs"],
    gen_reflections = 1,
    includes = ["//frc971/control_loops:control_loops_fbs_includes"],
)

flatbuffer_cc_library(
    name = "trajectory_fbs",
    srcs = ["trajectory.fbs"],
    gen_reflections = 1,
    includes = ["//frc971/control_loops:control_loops_fbs_includes"],
)

flatbuffer_ts_library(
    name = "drivetrain_status_ts_fbs",
    srcs = ["drivetrain_status.fbs"],
    includes = ["//frc971/control_loops:control_loops_fbs_includes"],
    target_compatible_with = ["@platforms//os:linux"],
)

genrule(
    name = "drivetrain_goal_float_fbs_generated",
    srcs = ["drivetrain_goal.fbs"],
    outs = ["drivetrain_goal_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
)

genrule(
    name = "drivetrain_position_float_fbs_generated",
    srcs = ["drivetrain_position.fbs"],
    outs = ["drivetrain_position_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
)

genrule(
    name = "drivetrain_output_float_fbs_generated",
    srcs = ["drivetrain_output.fbs"],
    outs = ["drivetrain_output_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
)

genrule(
    name = "drivetrain_status_float_fbs_generated",
    srcs = ["drivetrain_status.fbs"],
    outs = ["drivetrain_status_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
)

flatbuffer_cc_library(
    name = "drivetrain_goal_float_fbs",
    srcs = ["drivetrain_goal_float.fbs"],
    gen_reflections = 1,
    includes = [
        ":spline_goal_fbs_includes",
        "//frc971/control_loops:control_loops_fbs_includes",
    ],
)

flatbuffer_cc_library(
    name = "drivetrain_output_float_fbs",
    srcs = ["drivetrain_output_float.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "drivetrain_position_float_fbs",
    srcs = ["drivetrain_position_float.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "drivetrain_status_float_fbs",
    srcs = ["drivetrain_status_float.fbs"],
    gen_reflections = 1,
    includes = ["//frc971/control_loops:control_loops_fbs_includes"],
)

aos_config(
    name = "simulation_channels",
    src = "drivetrain_simulation_channels.json",
    flatbuffers = [
        ":drivetrain_status_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

aos_config(
    name = "simulation_config",
    src = "drivetrain_simulation_config.json",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":config",
        ":simulation_channels",
    ],
)

aos_config(
    name = "config",
    src = "drivetrain_config.json",
    flatbuffers = [
        ":drivetrain_goal_fbs",
        ":trajectory_fbs",
        ":spline_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":drivetrain_position_fbs",
        ":localizer_fbs",
        "//frc971/queues:gyro_fbs",
        "//frc971/queues:gyro_uid_fbs",
        "//frc971/wpilib:imu_fbs",
        "//frc971/wpilib:imu_batch_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
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
        "//frc971/control_loops:state_feedback_loop",
    ] + select({
        "@platforms//os:linux": ["//frc971/control_loops:hybrid_state_feedback_loop"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "hybrid_ekf",
    hdrs = ["hybrid_ekf.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        "//aos:math",
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "localizer",
    hdrs = ["localizer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_status_fbs",
        ":hybrid_ekf",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:pose",
    ],
)

cc_library(
    name = "gear",
    hdrs = [
        "gear.h",
    ],
)

cc_library(
    name = "splinedrivetrain",
    srcs = [
        "splinedrivetrain.cc",
    ],
    hdrs = [
        "splinedrivetrain.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":spline",
        ":spline_goal_fbs",
        ":trajectory",
        "//aos:condition",
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":localizer",
        ":spline_goal_fbs",
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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_states",
        ":drivetrain_status_fbs",
        ":gear",
        ":localizer",
        ":spline_goal_fbs",
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
    copts = select({
        "@platforms//os:none": ["-Wno-type-limits"],
        "//conditions:default": [],
    }),
    deps = [
        ":drivetrain_config",
        ":drivetrain_states",
        ":gear",
        "//aos:math",
        "//aos/controls:polytope",
        "//frc971/control_loops:coerce_goal",
        "//frc971/control_loops:control_loops_fbs",
        ":spline_goal_fbs",
        "//frc971/control_loops:state_feedback_loop",
    ] + select({
        "@platforms//os:linux": [
            ":drivetrain_goal_fbs",
            ":drivetrain_output_fbs",
            ":drivetrain_position_fbs",
            ":drivetrain_status_fbs",
            "//aos/robot_state:robot_state_fbs",
            "//aos/util:log_interval",
        ],
        "@platforms//os:none": [
            ":drivetrain_goal_float_fbs",
            ":drivetrain_output_float_fbs",
            ":drivetrain_position_float_fbs",
            ":drivetrain_status_float_fbs",
        ],
    }),
)

genrule(
    name = "genrule_down_estimator",
    outs = [
        "down_estimator.h",
        "down_estimator.cc",
    ],
    cmd = "$(location //frc971/control_loops/python:down_estimator) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "drivetrain_states",
    hdrs = ["drivetrain_states.h"],
)

cc_library(
    name = "drivetrain_lib",
    srcs = [
        "drivetrain.cc",
    ],
    hdrs = [
        "drivetrain.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":down_estimator",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_states",
        ":drivetrain_status_fbs",
        ":gear",
        ":improved_down_estimator",
        ":line_follow_drivetrain",
        ":localizer",
        ":localizer_fbs",
        ":polydrivetrain",
        ":spline_goal_fbs",
        ":splinedrivetrain",
        ":ssdrivetrain",
        "//aos/controls:control_loop",
        "//aos/util:log_interval",
        "//frc971/control_loops:runge_kutta",
        "//frc971/queues:gyro_fbs",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/wpilib:imu_fbs",
        "//frc971/zeroing:imu_zeroer",
    ],
)

cc_library(
    name = "drivetrain_test_lib",
    testonly = True,
    srcs = ["drivetrain_test_lib.cc"],
    hdrs = ["drivetrain_test_lib.h"],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    target_compatible_with = ["@platforms//os:linux"],
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
        "//frc971/queues:gyro_fbs",
        "//frc971/wpilib:imu_fbs",
        "//frc971/wpilib:imu_batch_fbs",
        "//y2016:constants",
        "//y2016/control_loops/drivetrain:polydrivetrain_plants",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)

cc_test(
    name = "drivetrain_lib_test",
    srcs = [
        "drivetrain_lib_test.cc",
    ],
    data = [":simulation_config"],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    linkstatic = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":trajectory_generator",
        ":drivetrain_lib",
        ":localizer_fbs",
        ":drivetrain_goal_fbs",
        ":drivetrain_status_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_test_lib",
        "//aos/controls:control_loop_test",
        "//aos/events/logging:log_writer",
        "//aos/testing:googletest",
        "//frc971/queues:gyro_fbs",
        "//frc971/wpilib:imu_fbs",
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
    tools = [
        "//frc971/control_loops/python:haptic_wheel",
    ],
    visibility = ["//visibility:private"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/control_loops:binomial",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "spline.so",
    srcs = ["libspline.cc"],
    linkshared = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        ":spline",
        ":trajectory",
        "//aos/logging:implementations",
        "//aos/network:team_number",
        "//y2020/control_loops/drivetrain:drivetrain_base",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "spline_test",
    srcs = [
        "spline_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":spline",
        ":trajectory_fbs",
        "//aos/logging",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:fixed_quadrature",
        "@com_github_google_glog//:glog",
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        "//aos/testing:googletest",
        "//aos:flatbuffers",
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        ":drivetrain_config",
        ":spline_goal_fbs",
        ":trajectory_fbs",
        "//aos/util:math",
        "//frc971/control_loops:c2d",
        "//frc971/control_loops:dlqr",
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:runge_kutta",
        "//frc971/control_loops:state_feedback_loop",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "trajectory_generator",
    srcs = ["trajectory_generator.cc"],
    hdrs = ["trajectory_generator.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        ":drivetrain_config",
        ":spline_goal_fbs",
        ":trajectory",
        ":trajectory_fbs",
    ],
)

cc_binary(
    name = "trajectory_plot",
    srcs = [
        "trajectory_plot.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
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

cc_library(
    name = "improved_down_estimator",
    srcs = [
        "improved_down_estimator.cc",
    ],
    hdrs = [
        "improved_down_estimator.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_status_fbs",
        "//aos/controls:quaternion_utils",
        "//aos/events:event_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:runge_kutta",
        "@//aos/time",
        "@com_github_google_glog//:glog",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "improved_down_estimator_test",
    srcs = [
        "improved_down_estimator_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_test_lib",
        "//aos/controls:quaternion_utils",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "//frc971/control_loops/drivetrain:improved_down_estimator",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "camera",
    srcs = ["camera.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/containers:sized_array",
        "//frc971/control_loops:pose",
    ],
)

cc_test(
    name = "camera_test",
    srcs = ["camera_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":camera",
        "//aos/testing:googletest",
    ],
)

ts_library(
    name = "down_estimator_plotter",
    srcs = ["down_estimator_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
        "//frc971/wpilib:imu_plot_utils",
    ],
)

ts_library(
    name = "drivetrain_plotter",
    srcs = ["drivetrain_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
        "//frc971/wpilib:imu_plot_utils",
    ],
)

ts_library(
    name = "robot_state_plotter",
    srcs = ["robot_state_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)
