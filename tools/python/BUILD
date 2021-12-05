load("@rules_python//python:defs.bzl", "py_runtime_pair")

py_runtime(
    name = "python3_runtime",
    files = [
        "runtime_binary.sh",
        "@python_repo//:all_files",
    ],
    interpreter = "runtime_binary.sh",
    python_version = "PY3",
)

py_runtime_pair(
    name = "py_runtime",
    py2_runtime = None,
    py3_runtime = ":python3_runtime",
)

toolchain(
    name = "python_toolchain",
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    exec_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    toolchain = ":py_runtime",
    toolchain_type = "@rules_python//python:toolchain_type",
)
