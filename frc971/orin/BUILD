load("//frc971:halide.bzl", "halide_library")

halide_library(
    name = "ycbcr",
    src = "crcv_generator.cc",
    args = "rows=1088 cols=1456 ystride=2048 cbcrstride=3840",
    function = "ycbcr",
    visibility = ["//visibility:public"],
)
