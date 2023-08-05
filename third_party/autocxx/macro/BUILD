load("@rules_rust//rust:defs.bzl", "rust_proc_macro")

package(default_visibility = ["//visibility:public"])

licenses([
    "notice",  # MIT from expression "MIT OR Apache-2.0"
])

rust_proc_macro(
    name = "autocxx_macro",
    srcs = glob(["**/*.rs"]),
    crate_root = "src/lib.rs",
    edition = "2021",
    rustc_flags = [
        "--cap-lints=allow",
    ],
    tags = [
        "cargo-raze",
        "crate-name=autocxx-macro",
        "manual",
    ],
    version = "0.16.0",
    deps = [
        "//third_party/autocxx/parser:autocxx_parser",
        "@crate_index//:proc-macro-error",
        "@crate_index//:proc-macro2",
        "@crate_index//:quote",
        "@crate_index//:syn",
    ],
)
