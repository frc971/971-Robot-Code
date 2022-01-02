cc_library(
    name = "print",
    hdrs = [
        "print.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/containers:sized_array",
        "//motors/core",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "uart",
    srcs = [
        "uart.cc",
    ],
    hdrs = [
        "uart.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        ":print",
        "//motors/core",
        "//motors/peripheral:uart",
    ],
)

cc_library(
    name = "itm",
    srcs = [
        "itm.cc",
    ],
    hdrs = [
        "itm.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        ":print",
        "//motors/core:itm",
    ],
)

cc_library(
    name = "semihosting",
    srcs = [
        "semihosting.cc",
    ],
    hdrs = [
        "semihosting.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        ":print",
        "//motors/core:semihosting",
    ],
)

cc_library(
    name = "usb",
    srcs = [
        "usb.cc",
    ],
    hdrs = [
        "usb.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        ":print",
        "//motors/core",
        "//motors/usb",
        "//motors/usb:cdc",
    ],
)
