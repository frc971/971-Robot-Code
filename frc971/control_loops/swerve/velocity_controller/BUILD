package(default_visibility = ["//visibility:public"])

py_library(
    name = "experience_buffer",
    srcs = [
        "experience_buffer.py",
    ],
    deps = [
        "@pip//flashbax",
        "@pip//jax",
    ],
)

py_test(
    name = "experience_buffer_test",
    srcs = [
        "experience_buffer_test.py",
    ],
    deps = [
        ":experience_buffer",
        "@pip//flashbax",
        "@pip//jax",
    ],
)

py_binary(
    name = "main",
    srcs = [
        "main.py",
        "model.py",
        "physics.py",
        "train.py",
    ],
    deps = [
        ":experience_buffer",
        "//frc971/control_loops/swerve:jax_dynamics",
        "@pip//absl_py",
        "@pip//aim",
        "@pip//clu",
        "@pip//flashbax",
        "@pip//flax",
        "@pip//jax",
        "@pip//jaxtyping",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//tensorflow",
    ],
)

py_binary(
    name = "lqr_plot",
    srcs = [
        "lqr_plot.py",
        "model.py",
        "physics.py",
    ],
    deps = [
        ":experience_buffer",
        "//frc971/control_loops/swerve:jax_dynamics",
        "@pip//absl_py",
        "@pip//flashbax",
        "@pip//flax",
        "@pip//jax",
        "@pip//jaxtyping",
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//pygobject",
        "@pip//tensorflow",
    ],
)
