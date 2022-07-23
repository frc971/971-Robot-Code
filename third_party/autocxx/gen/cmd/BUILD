load("@rules_rust//rust:defs.bzl", "rust_binary")

package(default_visibility = ["//visibility:public"])

licenses([
    "notice",  # MIT from expression "MIT OR Apache-2.0"
])

rust_binary(
    name = "gen",
    srcs = glob(["**/*.rs"]),
    crate_root = "src/main.rs",
    edition = "2021",
    rustc_flags = [
        "--cap-lints=allow",
    ],
    tags = [
        "cargo-raze",
        "crate-name=autocxx-gen",
        "manual",
    ],
    version = "0.16.0",
    deps = [
        "//third_party/autocxx/engine:autocxx_engine",
        "@//third_party/cargo:clap",
        "@//third_party/cargo:env_logger",
        "@//third_party/cargo:indexmap",
        "@//third_party/cargo:miette",
        "@//third_party/cargo:pathdiff",
        "@//third_party/cargo:proc_macro2",
    ],
)
