load("@rules_rust//rust:defs.bzl", "rust_library")

package(default_visibility = ["//visibility:public"])

licenses([
    "notice",  # MIT from expression "MIT OR Apache-2.0"
])

rust_library(
    name = "autocxx_parser",
    srcs = glob(["**/*.rs"]),
    crate_features = [
        "reproduction_case",
    ],
    crate_root = "src/lib.rs",
    edition = "2021",
    rustc_flags = [
        "--cap-lints=allow",
    ],
    tags = [
        "cargo-raze",
        "crate-name=autocxx-parser",
        "manual",
    ],
    version = "0.16.0",
    deps = [
        "@crate_index//:indexmap",
        "@crate_index//:itertools",
        "@crate_index//:log",
        "@crate_index//:once_cell",
        "@crate_index//:proc-macro2",
        "@crate_index//:quote",
        "@crate_index//:serde",
        "@crate_index//:serde_json",
        "@crate_index//:syn-2.0.28",
        "@crate_index//:thiserror",
    ],
)
