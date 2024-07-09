load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

cc_library(
    name = "static_flatbuffers",
    srcs = ["static_flatbuffers.cc"],
    hdrs = ["static_flatbuffers.h"],
    deps = [
        ":static_table",
        ":static_vector",
        "//aos:json_to_flatbuffer",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "base",
    srcs = [
        "base.cc",
    ],
    hdrs = [
        "base.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:shared_span",
        "//aos/containers:resizeable_buffer",
        "//aos/ipc_lib:data_alignment",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "base_test",
    srcs = ["base_test.cc"],
    deps = [
        ":base",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "static_table",
    hdrs = ["static_table.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":base",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "static_vector",
    hdrs = ["static_vector.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/containers:inlined_vector",
        "//aos/containers:sized_array",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "builder",
    hdrs = ["builder.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":static_table",
        "//aos:flatbuffers",
    ],
)

cc_static_flatbuffer(
    name = "test_schema",
    function = "aos::fbs::testing::TestTableSchema",
    target = ":test_fbs_fbs_reflection_out",
)

cc_static_flatbuffer(
    name = "interesting_schemas",
    function = "aos::fbs::testing::UnsupportedSchema",
    target = ":interesting_schemas_fbs_reflection_out",
)

flatbuffer_cc_library(
    name = "interesting_schemas_fbs",
    srcs = ["interesting_schemas.fbs"],
    gen_reflections = True,
)

cc_test(
    name = "static_flatbuffers_test",
    srcs = ["static_flatbuffers_test.cc"],
    data = ["//aos/flatbuffers/test_dir:test_data"],
    deps = [
        ":builder",
        ":interesting_schemas",
        ":static_flatbuffers",
        ":test_fbs",
        ":test_schema",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/flatbuffers/test_dir:include_reflection_fbs",
        "//aos/flatbuffers/test_dir:type_coverage_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "@com_github_google_flatbuffers//src:flatc_library",
    ],
)

cc_test(
    name = "static_flatbuffers_fuzz_test",
    # This does a combinatoric sweep over a variety of things, and can end up taking a while.
    timeout = "eternal",
    srcs = ["static_flatbuffers_fuzz_test.cc"],
    data = ["//aos/flatbuffers/test_dir:test_data"],
    deps = [
        ":builder",
        ":static_flatbuffers",
        ":test_fbs",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/flatbuffers/test_dir:type_coverage_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

cc_binary(
    name = "generate",
    srcs = ["generate.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":static_flatbuffers",
        "//aos:flatbuffers",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

sh_binary(
    name = "generate_wrapper",
    srcs = ["generate.sh"],
    data = [
        ":generate",
        "//:.clang-format",
        "@llvm_k8//:bin",
    ],
    visibility = ["//visibility:public"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)

static_flatbuffer(
    name = "test_fbs",
    srcs = ["test.fbs"],
    deps = ["//aos/flatbuffers/test_dir:include_fbs"],
)

filegroup(
    name = "test_static_file",
    srcs = ["test_static.h"],
    visibility = [":__subpackages__"],
)
