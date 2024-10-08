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
