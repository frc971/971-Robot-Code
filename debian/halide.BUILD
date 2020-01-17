cc_library(
    name = "halide",
    srcs = ["lib/libHalide.a"],
    hdrs = glob(["include/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "gengen",
    srcs = ["tools/GenGen.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":halide",
    ],
)

cc_library(
    name = "runtime",
    hdrs = [
        "include/HalideBuffer.h",
        "include/HalideRuntime.h",
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
)
