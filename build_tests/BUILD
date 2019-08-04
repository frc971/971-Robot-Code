load("//tools/build_rules:ruby.bzl", "ruby_binary")
load("//aos/build:queues.bzl", "queue_library")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("//tools/cpp/emscripten:defs.bzl", "emcc_binary")

emcc_binary(
    name = "helloworld.html",
    srcs = ["helloworld.cc"],
)

emcc_binary(
    name = "webgl.html",
    srcs = ["webgl_draw_triangle.c"],
)

emcc_binary(
    name = "webgl2.html",
    srcs = ["webgl2_draw_packed_triangle.c"],
    # Enable WEBGL2 (-s is used by the emscripten
    # compiler to specify sundry options).
    linkopts = [
        "-s",
        "USE_WEBGL2=1",
    ],
)

cc_test(
    name = "gflags_build_test",
    size = "small",
    srcs = [
        "gflags.cc",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

ruby_binary(
    name = "ruby_binary",
    srcs = [
        "ruby.rb",
        "ruby_to_require.rb",
    ],
    data = [
        "ruby_to_require.rb",
    ],
)

sh_test(
    name = "ruby_build_test",
    size = "small",
    srcs = [
        "ruby_check.sh",
    ],
    data = [
        ":ruby_binary",
    ],
)

queue_library(
    name = "queue_library",
    srcs = [
        "queue.q",
    ],
)

cc_test(
    name = "queue_build_test",
    size = "small",
    srcs = [
        "queue.cc",
    ],
    deps = [
        ":queue_library",
    ],
)

cc_binary(
    name = "tcmalloc_build_test_binary",
    srcs = [
        "tcmalloc.cc",
    ],
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
)

cc_proto_library(
    name = "proto_build_test_library",
    srcs = ["proto.proto"],
    deps = [
        ":proto_build_test_library_base",
        "@com_google_protobuf//:cc_wkt_protos",
    ],
)

cc_proto_library(
    name = "proto_build_test_library_base",
    srcs = ["proto_base.proto"],
)

cc_test(
    name = "proto_build_test",
    size = "small",
    srcs = [
        "proto.cc",
    ],
    deps = [
        ":proto_build_test_library",
        "//aos/testing:googletest",
    ],
)
