load("//tools/build_rules:js.bzl", "ts_project")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_go_library", "flatbuffer_py_library")
load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")
load("//tools/build_rules:apache.bzl", "apache_wrapper")
load("@rules_rust//rust:defs.bzl", "rust_binary", "rust_library", "rust_test")
load("@npm//@bazel/concatjs:index.bzl", "karma_web_test_suite")
load("//tools/build_rules:autocxx.bzl", "autocxx_library")

cc_test(
    name = "gflags_build_test",
    size = "small",
    srcs = [
        "gflags.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "tcmalloc_build_test_binary",
    srcs = [
        "tcmalloc.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

sh_test(
    name = "tcmalloc_build_test",
    size = "small",
    srcs = [
        "tcmalloc_test.sh",
    ],
    data = [
        ":tcmalloc_build_test_binary",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_proto_library(
    name = "proto_build_test_library",
    srcs = ["proto.proto"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":proto_build_test_library_base",
        "@com_google_protobuf//:cc_wkt_protos",
    ],
)

cc_proto_library(
    name = "proto_build_test_library_base",
    srcs = ["proto_base.proto"],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "proto_build_test",
    size = "small",
    srcs = [
        "proto.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":proto_build_test_library",
        "//aos/testing:googletest",
    ],
)

flatbuffer_py_library(
    name = "test_python_fbs",
    srcs = ["test.fbs"],
    namespace = "aos.examples",
    tables = [
        "Foo",
        "Bar",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_go_library(
    name = "test_go_fbs",
    srcs = ["test.fbs"],
    importpath = "github.com/frc971/971-Robot-Code/build_tests/fbs",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)

py_test(
    name = "python_fbs",
    srcs = ["python_fbs.py"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":test_python_fbs"],
)

py_test(
    name = "python3_opencv",
    srcs = ["python_opencv.py"],
    main = "python_opencv.py",
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["@pip//opencv_python"],
)

py_test(
    name = "python_jinja2",
    srcs = ["python_jinja2.py"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["@pip//jinja2"],
)

go_binary(
    name = "hello_go",
    embed = [":build_tests_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)

go_library(
    name = "build_tests_lib",
    srcs = [
        "hello.go",
        # Not sure why gazelle wants this here?
        "hello_autocxx.h",
    ],
    importpath = "github.com/frc971/971-Robot-Code/build_tests",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
    deps = ["//build_tests/go_greeter"],
)

py_binary(
    name = "dummy_http_server",
    srcs = ["dummy_http_server.py"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
)

apache_wrapper(
    name = "apache_https_demo",
    binary = ":dummy_http_server",
    target_compatible_with = ["@platforms//cpu:x86_64"],
)

rust_library(
    name = "hello_lib",
    srcs = ["hello_lib.rs"],
    target_compatible_with = ["//tools/platforms/rust:has_support"],
)

rust_test(
    name = "hello_lib_test",
    crate = ":hello_lib",
    target_compatible_with = ["//tools/platforms/rust:has_support"],
)

rust_binary(
    name = "rust_hello",
    srcs = ["rust_hello.rs"],
    target_compatible_with = ["//tools/platforms/rust:has_support"],
    deps = [":hello_lib"],
)

ts_project(
    name = "build_tests_ts",
    srcs = ["basic.ts"],
)

ts_project(
    name = "ts_test",
    testonly = True,
    srcs = ["basic_test.ts"],
    deps = [
        ":build_tests_ts",
        "@npm//@types/jasmine",
    ],
)

karma_web_test_suite(
    name = "karma",
    testonly = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":ts_test"],
)

rust_library(
    name = "rust_in_cc_rs",
    srcs = ["rust_in_cc.rs"],
    target_compatible_with = ["//tools/platforms/rust:has_support"],
)

cc_test(
    name = "rust_in_cc",
    srcs = ["rust_in_cc.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":rust_in_cc_rs"],
)

cc_library(
    name = "hello_autocxx_cc",
    hdrs = [
        "hello_autocxx.h",
    ],
)

autocxx_library(
    name = "hello_autocxx",
    srcs = ["hello_autocxx.rs"],
    libs = [":hello_autocxx_cc"],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    target_compatible_with = ["//tools/platforms/rust:has_support"],
)

rust_test(
    name = "hello_autocxx_test",
    crate = ":hello_autocxx",
    # TODO: Make Rust play happy with pic vs nopic. Details at:
    # https://github.com/bazelbuild/rules_rust/issues/118
    rustc_flags = ["-Crelocation-model=static"],
    target_compatible_with = ["//tools/platforms/rust:has_support"],
)

py_test(
    name = "upstream_python_test",
    srcs = [
        "upstream_python_test.py",
    ],
    deps = [
        "@pip//matplotlib",
        "@pip//numpy",
        "@pip//scipy",
    ],
)

py_binary(
    name = "pygobject_example",
    srcs = ["pygobject_example.py"],
    deps = [
        "@pip//pygobject",
    ],
)

py_binary(
    name = "matplotlib_example",
    srcs = ["matplotlib_example.py"],
    deps = [
        "@pip//matplotlib",
        "@pip//pygobject",
    ],
)
