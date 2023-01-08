load("@rules_python//python:defs.bzl", "py_runtime_pair")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")

# Invoke this via "bazel run //tools/python:requirements.update".
compile_pip_requirements(
    name = "requirements",
    requirements_in = "requirements.txt",
    requirements_txt = "requirements.lock.txt",
    tags = [
        # The test pings pypi.org to make sure that the lock file matches the
        # requirements file.
        "requires-network",
        # The test causes some packages' setup.py to be executed which can
        # execute arbitrary code. Sometimes they look for compilers etc. That's
        # not good for our hermeticity assumptions. Disable the test lock file
        # test for now.
        "manual",
    ],
)

py_runtime(
    name = "python3_runtime",
    files = [
        "runtime_binary.sh",
        "@python3_9_x86_64-unknown-linux-gnu//:files",
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
    exec_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    toolchain = ":py_runtime",
    toolchain_type = "@rules_python//python:toolchain_type",
)

py_binary(
    name = "mirror_pip_packages",
    srcs = ["mirror_pip_packages.py"],
    deps = [
        "@pip//pkginfo",
        "@pip//requests",
    ],
)
