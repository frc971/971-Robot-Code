load("@rules_rust//rust:defs.bzl", "rust_library")

package(default_visibility = ["//visibility:public"])

licenses([
    "notice",  # MIT from expression "MIT OR Apache-2.0"
])

rust_library(
    name = "autocxx",
    srcs = glob(["**/*.rs"]),
    compile_data = ["README.md"],
    crate_root = "src/lib.rs",
    edition = "2021",
    proc_macro_deps = [
        "//third_party/cargo:aquamarine",
        "//third_party/autocxx/macro:autocxx_macro",
    ],
    rustc_flags = [
        "--cap-lints=allow",
    ],
    tags = [
        "cargo-raze",
        "crate-name=autocxx",
        "manual",
    ],
    version = "0.16.0",
    deps = [
        "//third_party/cargo:cxx",
        "//third_party/cargo:moveit",
    ],
)
