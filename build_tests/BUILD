load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_py_library")

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

flatbuffer_py_library(
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

py_test(
    name = "python3_opencv",
    srcs = ["python_opencv.py"],
    default_python_version = "PY3",
    main = "python_opencv.py",
    srcs_version = "PY2AND3",
    deps = ["@opencv_contrib_nonfree_amd64//:python_opencv"],
)

py_test(
    name = "python_jinja2",
    srcs = ["python_jinja2.py"],
    srcs_version = "PY2AND3",
    deps = ["@python_jinja2"],
)
