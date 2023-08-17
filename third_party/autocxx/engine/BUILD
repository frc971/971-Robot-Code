load("@rules_rust//rust:defs.bzl", "rust_library")

package(default_visibility = ["//visibility:public"])

licenses([
    "notice",  # MIT from expression "MIT OR Apache-2.0"
])

rust_library(
    name = "autocxx_engine",
    srcs = glob(["**/*.rs"]),
    crate_features = [
        "default",
        "reproduction_case",
        "serde_json",
    ],
    crate_root = "src/lib.rs",
    edition = "2021",
    proc_macro_deps = [
        "@crate_index//:rustversion",
        "@crate_index//:indoc",
        "@crate_index//:aquamarine",
    ],
    rustc_flags = [
        "--cap-lints=allow",
    ],
    tags = [
        "cargo-raze",
        "crate-name=autocxx-engine",
        "manual",
    ],
    version = "0.16.0",
    deps = [
        "//third_party/autocxx/parser:autocxx_parser",
        "@crate_index//:autocxx-bindgen",
        "@crate_index//:cxx-gen",
        "@crate_index//:indexmap",
        "@crate_index//:itertools",
        "@crate_index//:log",
        "@crate_index//:miette",
        "@crate_index//:once_cell",
        "@crate_index//:prettyplease",
        "@crate_index//:proc-macro2",
        "@crate_index//:quote",
        "@crate_index//:regex",
        "@crate_index//:serde_json",
        "@crate_index//:syn-2.0.28",
        "@crate_index//:tempfile",
        "@crate_index//:thiserror",
    ],
)
