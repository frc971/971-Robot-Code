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
        "@//third_party/cargo:indexmap",
        "@//third_party/cargo:itertools",
        "@//third_party/cargo:log",
        "@//third_party/cargo:once_cell",
        "@//third_party/cargo:proc_macro2",
        "@//third_party/cargo:quote",
        "@//third_party/cargo:serde",
        "@//third_party/cargo:serde_json",
        "@//third_party/cargo:syn",
        "@//third_party/cargo:thiserror",
    ],
)
