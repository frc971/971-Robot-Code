licenses(["notice"])

load("@//tools:environments.bzl", "mcu_cpus")

package(
    default_visibility = ["//visibility:public"],
)

exports_files([
    "LICENSE",
])

# Public flatc library to compile flatbuffer files at runtime.
cc_library(
    name = "flatbuffers",
    copts = ["-Wno-cast-align"],
    hdrs = ["//:public_headers"],
    linkstatic = 1,
    strip_include_prefix = "/include",
    deps = ["//src:flatbuffers"],
)

# Public C++ headers for the Flatbuffers library.
filegroup(
    name = "public_headers",
    srcs = [
        "include/flatbuffers/base.h",
        "include/flatbuffers/code_generators.h",
        "include/flatbuffers/flatbuffers.h",
        "include/flatbuffers/flexbuffers.h",
        "include/flatbuffers/grpc.h",
        "include/flatbuffers/hash.h",
        "include/flatbuffers/idl.h",
        "include/flatbuffers/minireflect.h",
        "include/flatbuffers/reflection.h",
        "include/flatbuffers/reflection_generated.h",
        "include/flatbuffers/registry.h",
        "include/flatbuffers/stl_emulation.h",
        "include/flatbuffers/util.h",
    ],
)

# Public flatc compiler library.
cc_library(
    name = "flatc_library",
    linkstatic = 1,
    deps = [
        "//src:flatc_library",
    ],
)

# Public flatc compiler.
cc_binary(
    name = "flatc",
    data = [
        "reflection/reflection.fbs",
    ],
    deps = [
        "//src:flatc",
    ],
)

filegroup(
    name = "flatc_headers",
    srcs = [
        "include/flatbuffers/flatc.h",
    ],
    visibility = ["//:__subpackages__"],
)

# Library used by flatbuffer_cc_library rules.
cc_library(
    name = "runtime_cc",
    hdrs = [
        "include/flatbuffers/base.h",
        "include/flatbuffers/flatbuffers.h",
        "include/flatbuffers/flexbuffers.h",
        "include/flatbuffers/stl_emulation.h",
        "include/flatbuffers/util.h",
    ],
    compatible_with = mcu_cpus,
    linkstatic = 1,
    strip_include_prefix = "/include",
)
py_library(
    name = "flatpy",
    srcs = glob(["python/flatbuffers/*.py"]),
    imports = ["python/"],
)

filegroup(
    name = "flatjs",
    srcs = ["js/flatbuffers.js"],
)
