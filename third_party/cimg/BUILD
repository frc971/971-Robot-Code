licenses(["notice"])

cc_library(
    name = "CImg",
    hdrs = glob([
        "CImg.h",
        "plugins/*.h",
    ]),
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/libjpeg",
    ],
)
