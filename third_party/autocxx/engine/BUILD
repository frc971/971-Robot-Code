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
        "//third_party/cargo:indoc",
        "//third_party/cargo:aquamarine",
        "//third_party/cargo:strum_macros",
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
        "//third_party/cargo:autocxx_bindgen",
        "//third_party/cargo:cxx_gen",
        "//third_party/cargo:indexmap",
        "//third_party/cargo:itertools",
        "//third_party/cargo:log",
        "//third_party/cargo:miette",
        "//third_party/cargo:once_cell",
        "//third_party/cargo:proc_macro2",
        "//third_party/cargo:quote",
        "//third_party/cargo:regex",
        "//third_party/cargo:serde_json",
        "//third_party/cargo:syn",
        "//third_party/cargo:tempfile",
        "//third_party/cargo:thiserror",
        "//third_party/cargo:version_check",
    ],
)
