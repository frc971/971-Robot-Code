load("@bazel_skylib//rules:write_file.bzl", "write_file")
load("@build_bazel_rules_nodejs//toolchains/node:node_toolchain.bzl", "node_toolchain")

write_file(
    name = "noop_error_exit",
    out = "noop_error_exit.sh",
    content = [
        "#!/bin/bash",
        "echo 'This should never be executed. Something went wrong.' >&2",
        "echo 'This NOOP NodeJs toolchain should never be executed. Something went wrong.' >&2",
        "echo 'Check that your target has `target_compatible_with` set to a platform that supports NodeJs.' >&2",
        "exit 1",
    ],
    is_executable = True,
)

node_toolchain(
    name = "noop_node_toolchain_impl",
    target_tool = ":noop_error_exit",
)

toolchain(
    name = "noop_node_toolchain",
    exec_compatible_with = [
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "//tools/platforms/nodejs:lacks_support",
    ],
    toolchain = ":noop_node_toolchain_impl",
    toolchain_type = "@build_bazel_rules_nodejs//toolchains/node:toolchain_type",
)
