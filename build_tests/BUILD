load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("//tools/cpp/emscripten:defs.bzl", "emcc_binary")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_python_library")

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
    srcs = ["webgl2_benchmark.cc"],
    html_shell = "minimal_shell.html",
    # Enable WEBGL2 (-s is used by the emscripten
    # compiler to specify sundry options).
    linkopts = [
        "-s",
        "USE_WEBGL2=1",
        "-s",
        "TOTAL_MEMORY=" + repr(256 * 1024 * 1024),
    ],
)

emcc_binary(
    name = "plotter.html",
    srcs = ["webgl2_plot_test.cc"],
    html_shell = "minimal_shell.html",
    linkopts = [
        "-s",
        "USE_WEBGL2=1",
        "-s",
        "FULL_ES3=1",
        "-s",
        "TOTAL_MEMORY=" + repr(256 * 1024 * 1024),
    ],
    deps = [
        "//frc971/analysis/plotting:webgl2_animator",
        "//frc971/analysis/plotting:webgl2_plotter",
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

flatbuffer_python_library(
    name = "test_python_fbs",
    srcs = ["test.fbs"],
    namespace = "aos.examples",
    tables = [
        "Foo",
        "Bar",
    ],
    visibility = ["//visibility:public"],
)

py_test(
    name = "python_fbs",
    srcs = ["python_fbs.py"],
    deps = [":test_python_fbs"],
)
