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

def rust_doc(target_compatible_with = ["//tools/platforms/rust:has_support"], rustdoc_flags = ["-Dwarnings"], **kwargs):
    _rust_doc(
        target_compatible_with = target_compatible_with,
        rustdoc_flags = rustdoc_flags,
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

def rust_test(target_compatible_with = ["//tools/platforms/rust:has_support"], rustc_flags = [], **kwargs):
    _rust_test(
        target_compatible_with = select({
            Label("//conditions:default"): target_compatible_with,
            Label("//tools:has_msan"): ["@platforms//:incompatible"],
        }),
        rustc_flags = rustc_flags + ["-Crelocation-model=static"],
        **kwargs
    )

def rust_library(
        name,
        target_compatible_with = ["//tools/platforms/rust:has_support"],
        gen_docs = True,
        gen_tests = True,
        gen_doctests = True,
        **kwargs):
    test_params = {}
    doctest_params = {}
    params = {}

    for (param, value) in kwargs.items():
        if param.startswith("test_"):
            test_params[param[5:]] = value
        elif param.startswith("doctest_"):
            doctest_params[param[8:]] = value
        else:
            params[param] = value

    _rust_library(
        name = name,
        target_compatible_with = select({
            Label("//conditions:default"): target_compatible_with,
            Label("//tools:has_msan"): ["@platforms//:incompatible"],
        }),
        **params
    )

    if gen_tests:
        rust_test(
            name = name + "_test",
            crate = name,
            **test_params
        )

    if gen_docs:
        rust_doc(
            name = name + "_doc",
            crate = name,
            target_compatible_with = ["//tools/platforms/rust:has_support"],
            rustdoc_flags = ["--document-private-items", "-Dwarnings"],
        )

    if gen_doctests:
        rust_doc_test(
            name = name + "_doctest",
            crate = name,
            **doctest_params
        )

def flatbuffer_rust_library(target_compatible_with = ["//tools/platforms/rust:has_support"], **kwargs):
    _flatbuffer_rust_library(
        target_compatible_with = select({
            Label("//conditions:default"): target_compatible_with,
            Label("//tools:has_msan"): ["@platforms//:incompatible"],
        }),
        **kwargs
    )
