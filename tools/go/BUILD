py_library(
    name = "mirror_lib",
    srcs = [
        "mirror_lib.py",
    ],
)

py_binary(
    name = "tweak_gazelle_go_deps",
    srcs = [
        "tweak_gazelle_go_deps.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":mirror_lib",
    ],
)

py_binary(
    name = "mirror_go_repos",
    srcs = [
        "mirror_go_repos.py",
    ],
    data = [
        "@go_sdk//:bin/go",
    ],
    deps = [
        ":mirror_lib",
    ],
)

# The remainder of this file exists to create a NOOP toolchain for Go on
# platforms that don't support Go. We can probably get rid of this once
# https://github.com/bazelbuild/bazel/issues/12897 is fixed.
#
# For platforms that do support Go, we use go_register_toolchain() in
# WORKSPACE.

load("@bazel_skylib//rules:write_file.bzl", "write_file")
load("@io_bazel_rules_go//go:def.bzl", "go_sdk", "go_toolchain")

write_file(
    name = "noop_error_exit",
    out = "noop_error_exit.sh",
    content = [
        "#!/bin/bash",
        "echo 'This should never be executed. Something went wrong.' >&2",
        "echo 'This NOOP Go toolchain should never be executed. Something went wrong.' >&2",
        "echo 'Check that your target has `target_compatible_with` set to a platform that supports Go.' >&2",
        "exit 1",
    ],
    is_executable = True,
)

go_sdk(
    name = "noop_sdk",
    go = ":noop_error_exit",
    goarch = "none",
    goos = "none",
    root_file = "NOOP_FILE_THAT_DOES_NOT_EXIST",
)

go_toolchain(
    name = "noop_go_toolchain_impl",
    builder = ":noop_error_exit",
    cgo_link_flags = None,
    goarch = "none",
    goos = "none",
    link_flags = None,
    sdk = ":noop_sdk",
    tags = ["manual"],
)

toolchain(
    name = "noop_go_toolchain",
    exec_compatible_with = [
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "//tools/platforms/go:lacks_support",
    ],
    toolchain = ":noop_go_toolchain_impl",
    toolchain_type = "@io_bazel_rules_go//go:toolchain",
)
