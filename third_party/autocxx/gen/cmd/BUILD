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
        "@crate_index//:clap",
        "@crate_index//:env_logger",
        "@crate_index//:indexmap",
        "@crate_index//:miette",
        "@crate_index//:pathdiff",
        "@crate_index//:proc-macro2",
    ],
)
