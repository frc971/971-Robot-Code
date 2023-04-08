cc_library(
    name = "tensorflow-k8",
    srcs = ["k8/libtensorflowlite.so"],
    hdrs = glob(["include/**/*.h"]),
    copts = ["-Wno-unused-parameter"],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "tensorflow-arm",
    srcs = ["arm/libtensorflowlite.so"],
    hdrs = glob(["include/**/*.h"]),
    copts = ["-Wno-unused-parameter"],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

exports_files(["arm/libtensorflowlite.so"])
