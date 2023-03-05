cc_library(
    visibility = ["//visibility:public"],
    name = "tensorflow-k8",
    hdrs = glob(["include/**/*.h"]),
    strip_include_prefix = "include",
    srcs = ["k8/libtensorflowlite.so"]
)

cc_library(
    visibility = ["//visibility:public"],
    name = "tensorflow-arm",
    hdrs = glob(["include/**/*.h"]),
    strip_include_prefix = "include",
    srcs = ["arm/libtensorflowlite.so"]
)

