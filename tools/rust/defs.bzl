load(
    "@rules_rust//rust:defs.bzl",
    _rust_binary = "rust_binary",
    _rust_doc = "rust_doc",
    _rust_doc_test = "rust_doc_test",
    _rust_library = "rust_library",
    _rust_test = "rust_test",
)
load("@com_github_google_flatbuffers//:build_defs.bzl", _flatbuffer_rust_library = "flatbuffer_rust_library")

def rust_doc_test(target_compatible_with = ["//tools/platforms/rust:has_support"], tags = [], **kwargs):
    # TODO(james): Attempting to execute this remotely results
    # in complaints about overly large files.
    _rust_doc_test(
        tags = tags + ["no-remote-exec"],
        target_compatible_with = target_compatible_with,
        **kwargs
    )

def rust_doc(target_compatible_with = ["//tools/platforms/rust:has_support"], **kwargs):
    _rust_doc(
        target_compatible_with = target_compatible_with,
        **kwargs
    )

def rust_binary(target_compatible_with = ["//tools/platforms/rust:has_support"], rustc_flags = [], **kwargs):
    _rust_binary(
        target_compatible_with = select({
            Label("//conditions:default"): target_compatible_with,
            Label("//tools:has_msan"): ["@platforms//:incompatible"],
        }),
        # TODO: Make Rust play happy with pic vs nopic. Details at:
        # https://github.com/bazelbuild/rules_rust/issues/118
        rustc_flags = rustc_flags + ["-Crelocation-model=static"],
        **kwargs
    )

def rust_library(target_compatible_with = ["//tools/platforms/rust:has_support"], docs = True, **kwargs):
    _rust_library(
        target_compatible_with = select({
            Label("//conditions:default"): target_compatible_with,
            Label("//tools:has_msan"): ["@platforms//:incompatible"],
        }),
        **kwargs
    )

    if docs:
        rust_doc(
            name = kwargs["name"] + "_doc",
            crate = kwargs["name"],
            rustdoc_flags = ["--document-private-items"],
            target_compatible_with = ["//tools/platforms/rust:has_support"],
        )

def rust_test(target_compatible_with = ["//tools/platforms/rust:has_support"], rustc_flags = [], **kwargs):
    _rust_test(
        target_compatible_with = select({
            Label("//conditions:default"): target_compatible_with,
            Label("//tools:has_msan"): ["@platforms//:incompatible"],
        }),
        rustc_flags = rustc_flags + ["-Crelocation-model=static"],
        **kwargs
    )

def flatbuffer_rust_library(target_compatible_with = ["//tools/platforms/rust:has_support"], **kwargs):
    _flatbuffer_rust_library(
        target_compatible_with = select({
            Label("//conditions:default"): target_compatible_with,
            Label("//tools:has_msan"): ["@platforms//:incompatible"],
        }),
        **kwargs
    )
