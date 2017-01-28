#!/usr/bin/python3
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "overlay",
    hdrs = ["overlay.h"],
    deps = [
        '//aos/vision/math:vector',
        '//aos/vision/math:segment',
        '//aos/vision/image:image_types',
        ],
)

cc_library(name = "debug_viewer",
    srcs = ["debug_viewer.cc"],
    hdrs = ["debug_viewer.h"],
    deps = [
        '@usr_repo//:gtk+-3.0',
        "//aos/vision/image:image_types",
        ":overlay",
    ]
)
