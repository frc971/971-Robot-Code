cc_library(
    name = "libusb",
    srcs = [
        "usr/lib/x86_64-linux-gnu/libusb.so",
    ],
    hdrs = [
        "usr/include/usb.h",
    ],
    includes = [
        "usr/include",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libusb_1_0",
    srcs = [
        "usr/lib/x86_64-linux-gnu/libusb-1.0.so",
    ],
    hdrs = [
        "usr/include/libusb-1.0/libusb.h",
    ],
    includes = [
        "usr/include",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)
