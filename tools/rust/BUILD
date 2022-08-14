load("@rules_rust//rust:toolchain.bzl", "rust_stdlib_filegroup", "rust_toolchain")
load("@bazel_skylib//rules:write_file.bzl", "write_file")
load("@rules_rust//rust:defs.bzl", "rust_binary")

# We have to declare our toolchains individually to get the corect constraints
# configured so we can robustly select the correct one for each of our
# platforms.

toolchain(
    name = "rust-toolchain-x86",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    toolchain = "@rust__x86_64-unknown-linux-gnu_tools//:rust_toolchain",
    toolchain_type = "@rules_rust//rust:toolchain",
)

toolchain(
    name = "rust-toolchain-armv7",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:armv7",
        # Include this so we're incompatible with the roborio platform, to avoid
        # subtle order dependencies.
        "//tools/platforms/hardware:raspberry_pi",
    ],
    toolchain = "@rust__armv7-unknown-linux-gnueabihf_tools//:rust_toolchain",
    toolchain_type = "@rules_rust//rust:toolchain",
)

toolchain(
    name = "rust-toolchain-arm64",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:arm64",
    ],
    toolchain = "@rust__aarch64-unknown-linux-gnu_tools//:rust_toolchain",
    toolchain_type = "@rules_rust//rust:toolchain",
)

toolchain(
    name = "rust-toolchain-roborio",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:armv7",
        "//tools/platforms/hardware:roborio",
    ],
    toolchain = "@rust__arm-unknown-linux-gnueabi_tools//:rust_toolchain",
    toolchain_type = "@rules_rust//rust:toolchain",
)

# The remainder of this file exists to create a NOOP toolchain for Rust on
# platforms that don't support Rust. We can probably get rid of this once
# https://github.com/bazelbuild/bazel/issues/12897 is fixed.

write_file(
    name = "noop_error_exit",
    out = "noop_error_exit.sh",
    content = [
        "#!/bin/bash",
        "echo 'This should never be executed. Something went wrong.' >&2",
        "echo 'This NOOP Rust toolchain should never be executed. Something went wrong.' >&2",
        "echo 'Check that your target has `target_compatible_with` set to a platform that supports Rust.' >&2",
        "exit 1",
    ],
    is_executable = True,
)

rust_stdlib_filegroup(
    name = "empty_stdlib",
    srcs = [":noop_error_exit"],
)

rust_toolchain(
    name = "noop_rust_toolchain_impl",
    binary_ext = "",
    cargo = ":noop_error_exit",
    clippy_driver = ":noop_error_exit",
    default_edition = "2021",
    dylib_ext = ".so",
    exec_triple = "none",
    os = "none",
    rust_doc = ":noop_error_exit",
    rust_std = ":empty_stdlib",
    rustc = ":noop_error_exit",
    rustc_lib = ":noop_error_exit",
    rustc_srcs = None,
    rustfmt = ":noop_error_exit",
    staticlib_ext = ".a",
    stdlib_linkflags = [],
    tags = ["manual"],
    target_triple = "none",
)

toolchain(
    name = "noop_rust_toolchain",
    exec_compatible_with = [
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "//tools/platforms/rust:lacks_support",
    ],
    toolchain = ":noop_rust_toolchain_impl",
    toolchain_type = "@rules_rust//rust:toolchain",
)

rust_binary(
    name = "tweak_cargo_raze_output",
    srcs = ["tweak_cargo_raze_output.rs"],
    target_compatible_with = ["//tools/platforms/rust:has_support"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/cargo:anyhow",
        "//third_party/cargo:toml",
    ],
)

cc_library(
    name = "forward_allocator",
    srcs = ["forward_allocator.c"],
    visibility = ["//visibility:public"],
)
