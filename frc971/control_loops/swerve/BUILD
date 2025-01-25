load("@aspect_bazel_lib//lib:run_binary.bzl", "run_binary")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "swerve_zeroing_fbs",
    srcs = ["swerve_zeroing.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/zeroing:constants_fbs",
    ],
)

static_flatbuffer(
    name = "swerve_drivetrain_joystick_goal_fbs",
    srcs = ["swerve_drivetrain_joystick_goal.fbs"],
)

static_flatbuffer(
    name = "swerve_drivetrain_goal_fbs",
    srcs = ["swerve_drivetrain_goal.fbs"],
    deps = [
        ":swerve_drivetrain_joystick_goal_fbs",
        "//frc971/math:matrix_fbs",
    ],
)

static_flatbuffer(
    name = "swerve_drivetrain_status_fbs",
    srcs = ["swerve_drivetrain_status.fbs"],
    deps = [
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
    ],
)

static_flatbuffer(
    name = "swerve_drivetrain_output_fbs",
    srcs = ["swerve_drivetrain_output.fbs"],
    deps = ["//frc971/control_loops:can_talonfx_fbs"],
)

static_flatbuffer(
    name = "swerve_drivetrain_can_position_fbs",
    srcs = ["swerve_drivetrain_can_position.fbs"],
    deps = ["//frc971/control_loops:can_talonfx_fbs"],
)

static_flatbuffer(
    name = "swerve_drivetrain_position_fbs",
    srcs = ["swerve_drivetrain_position.fbs"],
    deps = ["//frc971/control_loops:control_loops_fbs"],
)

static_flatbuffer(
    name = "swerve_trajectory_fbs",
    srcs = ["swerve_trajectory.fbs"],
)

static_flatbuffer(
    name = "autonomous_init_fbs",
    srcs = ["autonomous_init.fbs"],
)

py_binary(
    name = "simulation",
    srcs = [
        "simulation.py",
    ],
    deps = [
        "//frc971/control_loops/python:controls",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//sympy",
    ],
)

cc_library(
    name = "swerve_control_loops",
    srcs = ["swerve_control_loops.cc"],
    hdrs = ["swerve_control_loops.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":inverse_kinematics",
        ":linear_velocity_controller",
        ":naive_estimator",
        ":swerve_drivetrain_can_position_fbs",
        ":swerve_drivetrain_goal_fbs",
        ":swerve_drivetrain_output_fbs",
        ":swerve_drivetrain_position_fbs",
        ":swerve_drivetrain_status_fbs",
        ":swerve_zeroing_fbs",
        ":velocity_ekf",
        "//aos:math",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//frc971/queues:gyro_fbs",
        "//frc971/wpilib:imu_batch_fbs",
        "//frc971/zeroing:continuous_absolute_encoder",
        "//frc971/zeroing:imu_zeroer",
    ],
)

cc_test(
    name = "velocity_ekf_test",
    srcs = ["velocity_ekf_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":velocity_ekf",
        "//aos/testing:googletest",
        "//frc971/math:eigen_matchers",
    ],
)

cc_test(
    name = "swerve_control_test",
    srcs = ["swerve_control_test.cc"],
    data = [
        ":aos_config",
        "//frc971/control_loops/swerve/test_module:rotation_json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":swerve_control_loops",
        ":swerve_drivetrain_can_position_fbs",
        ":swerve_drivetrain_goal_fbs",
        ":swerve_drivetrain_output_fbs",
        ":swerve_drivetrain_position_fbs",
        ":swerve_drivetrain_status_fbs",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

aos_config(
    name = "aos_config",
    src = "swerve.json",
    flatbuffers = [
        ":swerve_drivetrain_goal_fbs",
        ":swerve_drivetrain_output_fbs",
        ":swerve_drivetrain_position_fbs",
        ":swerve_drivetrain_can_position_fbs",
        ":swerve_drivetrain_status_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//frc971/input:aos_config"],
)

cc_library(
    name = "motors",
    hdrs = [
        "motors.h",
    ],
    deps = [
        "@symengine",
    ],
)

cc_binary(
    name = "generate_physics",
    srcs = [
        "generate_physics.cc",
    ],
    deps = [
        ":motors",
        "//aos:init",
        "//aos/util:file",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@symengine",
    ],
)

run_binary(
    name = "dynamics_codegen_cc",
    outs = [
        "dynamics.cc",
        "dynamics.h",
        "dynamics.py",
        "dynamics_constants.py",
    ],
    args = [
        "--output_base=$(BINDIR)/",
        "--cc_output_path=$(location :dynamics.cc)",
        "--h_output_path=$(location :dynamics.h)",
        "--casadi_py_output_path=$(location :dynamics.py)",
        "--constants_output_path=$(location :dynamics_constants.py)",
    ],
    tool = ":generate_physics",
)

run_binary(
    name = "dynamics_codegen_nocaster",
    outs = [
        "nocaster_dynamics.py",
    ],
    args = [
        "--caster=0.0",
        "--output_base=$(BINDIR)/",
        "--casadi_py_output_path=$(location :nocaster_dynamics.py)",
    ],
    tool = ":generate_physics",
)

run_binary(
    name = "dynamics_codegen_bigcaster",
    outs = [
        "bigcaster_dynamics.py",
    ],
    args = [
        "--caster=0.05",
        "--output_base=$(BINDIR)/",
        "--casadi_py_output_path=$(location :bigcaster_dynamics.py)",
    ],
    tool = ":generate_physics",
)

cc_library(
    name = "eigen_dynamics",
    srcs = ["dynamics.cc"],
    hdrs = ["dynamics.h"],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ],
)

py_library(
    name = "dynamics",
    srcs = [
        "bigcaster_dynamics.py",
        "dynamics.py",
        "nocaster_dynamics.py",
    ],
    deps = [
        "@pip//casadi",
    ],
)

py_library(
    name = "jax_dynamics",
    srcs = [
        "dynamics_constants.py",
        "jax_dynamics.py",
    ],
    deps = [
        "//frc971/control_loops/python:controls",
        "@pip//jax",
    ],
)

py_test(
    name = "physics_test_cpu",
    srcs = [
        "physics_test.py",
    ],
    data = [":cpp_dynamics.so"],
    env = {
        "JAX_PLATFORMS": "cpu",
    },
    main = "physics_test.py",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":casadi_velocity_mpc_lib",
        ":dynamics",
        ":jax_dynamics",
        ":physics_test_utils",
        "@pip//absl_py",
        "@pip//casadi",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//scipy",
    ],
)

py_test(
    name = "physics_test_gpu",
    srcs = [
        "physics_test.py",
    ],
    data = [":cpp_dynamics.so"],
    env = {
        "JAX_PLATFORMS": "cuda",
    },
    main = "physics_test.py",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":casadi_velocity_mpc_lib",
        ":dynamics",
        ":jax_dynamics",
        ":physics_test_utils",
        "@pip//absl_py",
        "@pip//casadi",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//scipy",
    ],
)

py_binary(
    name = "casadi_velocity_mpc_lib",
    srcs = [
        "casadi_velocity_mpc_lib.py",
    ],
    deps = [
        ":dynamics",
        "@pip//casadi",
        "@pip//numpy",
    ],
)

py_binary(
    name = "casadi_velocity_mpc",
    srcs = [
        "casadi_velocity_mpc.py",
    ],
    deps = [
        ":casadi_velocity_mpc_lib",
        "@pip//absl_py",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//scipy",
    ],
)

py_binary(
    name = "experience_collector",
    srcs = [
        "experience_collector.py",
    ],
    deps = [
        ":casadi_velocity_mpc_lib",
        ":jax_dynamics",
        "//frc971/control_loops/swerve/velocity_controller:physics",
        "@pip//absl_py",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//scipy",
        "@pip//tensorflow",
    ],
)

py_binary(
    name = "multi_experience_collector",
    srcs = ["multi_experience_collector.py"],
    data = [":experience_collector"],
    deps = [
        "@pip//absl_py",
    ],
)

py_library(
    name = "physics_test_utils",
    srcs = [
        "physics_test_utils.py",
    ],
    deps = [
        ":dynamics",
        "@pip//casadi",
        "@pip//numpy",
    ],
)

py_binary(
    name = "physics_debug",
    srcs = [
        "physics_debug.py",
    ],
    deps = [
        ":physics_test_utils",
        "@pip//matplotlib",
        "@pip//pygobject",
        "@pip//scipy",
    ],
)

py_binary(
    name = "debug_fatrop",
    srcs = [
        "debug_fatrop.py",
    ],
    deps = [
        "@pip//casadi",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//scipy",
    ],
)

py_binary(
    name = "smooth_function_graph",
    srcs = [
        "smooth_function_graph.py",
    ],
    deps = [
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//scipy",
    ],
)

py_binary(
    name = "multi_casadi_velocity_mpc",
    srcs = ["multi_casadi_velocity_mpc.py"],
    data = [":casadi_velocity_mpc"],
    deps = [
        ":dynamics",
        "@pip//absl_py",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
    ],
)

cc_binary(
    name = "cpp_dynamics.so",
    # Just use the python dynamics directly if you want them; this is just for testing.
    testonly = True,
    srcs = ["dynamics_python_bindings.cc"],
    linkshared = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":eigen_dynamics",
        "//third_party/python",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "linearization_utils",
    hdrs = ["linearization_utils.h"],
    deps = ["//frc971/control_loops:jacobian"],
)

cc_library(
    name = "linearized_controller",
    srcs = ["linearized_controller.cc"],
    hdrs = ["linearized_controller.h"],
    deps = [
        ":eigen_dynamics",
        ":linearization_utils",
        "//frc971/control_loops:c2d",
        "//frc971/control_loops:dlqr",
        "//frc971/control_loops:jacobian",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "linearized_controller_test",
    srcs = ["linearized_controller_test.cc"],
    deps = [
        ":linearized_controller",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "linear_velocity_controller",
    srcs = ["linear_velocity_controller.cc"],
    hdrs = ["linear_velocity_controller.h"],
    deps = [
        ":eigen_dynamics",
        ":inverse_kinematics",
        ":linearized_controller",
        ":simplified_dynamics",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_test(
    name = "linear_velocity_controller_test",
    srcs = ["linear_velocity_controller_test.cc"],
    deps = [
        ":linear_velocity_controller",
        "//aos:realtime",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "auto_diff_jacobian",
    hdrs = ["auto_diff_jacobian.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_google_ceres_solver//:ceres",
    ],
)

cc_test(
    name = "auto_diff_jacobian_test",
    srcs = ["auto_diff_jacobian_test.cc"],
    deps = [
        ":auto_diff_jacobian",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "simplified_dynamics",
    hdrs = ["simplified_dynamics.h"],
    deps = [
        ":auto_diff_jacobian",
        ":eigen_dynamics",
        ":linearization_utils",
        ":motors",
        "//aos/util:math",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "simplified_dynamics_test",
    srcs = ["simplified_dynamics_test.cc"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":simplified_dynamics",
        "//aos/testing:googletest",
        "//aos/time",
        "//frc971/control_loops:jacobian",
        "@com_google_absl//absl/log",
    ],
)

cc_library(
    name = "inverse_kinematics",
    hdrs = ["inverse_kinematics.h"],
    deps = [
        ":simplified_dynamics",
        "//aos/util:math",
    ],
)

cc_test(
    name = "inverse_kinematics_test",
    srcs = ["inverse_kinematics_test.cc"],
    deps = [
        ":inverse_kinematics",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "naive_estimator",
    srcs = ["naive_estimator.cc"],
    hdrs = ["naive_estimator.h"],
    deps = [
        ":autonomous_init_fbs",
        ":simplified_dynamics",
        ":swerve_drivetrain_can_position_fbs",
        ":swerve_drivetrain_position_fbs",
        ":swerve_drivetrain_status_fbs",
        ":swerve_zeroing_fbs",
        "//aos/events:event_loop",
        "//frc971/math:flatbuffers_matrix",
        "//frc971/zeroing:continuous_absolute_encoder",
    ],
)

cc_library(
    name = "velocity_ekf",
    srcs = ["velocity_ekf.cc"],
    hdrs = ["velocity_ekf.h"],
    deps = [
        ":linearization_utils",
        ":simplified_dynamics",
        ":swerve_drivetrain_status_fbs",
        "//frc971/estimation:ekf",
        "//frc971/math:flatbuffers_matrix",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "autonomous_controller",
    srcs = ["autonomous_controller.cc"],
    hdrs = ["autonomous_controller.h"],
    deps = [
        ":autonomous_init_fbs",
        ":simplified_dynamics",
        ":swerve_drivetrain_goal_fbs",
        ":swerve_drivetrain_status_fbs",
        ":swerve_trajectory_fbs",
        "//aos/events:event_loop",
        "//frc971/input:joystick_state_fbs",
        "//frc971/math:flatbuffers_matrix",
    ],
)

cc_binary(
    name = "gen_trajectory_flatbuffer",
    srcs = [
        "gen_trajectory_flatbuffer.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":swerve_trajectory_fbs",
        "//aos:init",
        "//aos/events:event_loop",
        "//aos/logging",
        "@com_google_absl//absl/flags:flag",
    ],
)
