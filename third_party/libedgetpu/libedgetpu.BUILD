cc_library(
    name = "libedgetpu-k8",
    srcs = ["k8/libedgetpu.so.1.0"],
    hdrs = glob(["include/**/*.h"]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libedgetpu-arm",
    srcs = ["arm/libedgetpu.so.1.0"],
    hdrs = glob(["include/**/*.h"]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

genrule(
    name = "renamed_libedgetpu-arm",
    srcs = [
        "arm/libedgetpu.so.1.0",
    ],
    outs = [
        "arm/libedgetpu.so.1",
    ],
    cmd = "cp $< $@",
    visibility = ["//visibility:public"],
)
