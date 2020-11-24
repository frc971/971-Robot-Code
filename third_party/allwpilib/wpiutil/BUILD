licenses(["notice"])

cc_library(
    name = "wpiutil",
    srcs = glob(
        [
            "src/main/native/cpp/llvm/*.cpp",
        ],
    ) + [
        "src/main/native/cpp/llvm/Unix/Path.inc",
        "src/main/native/cpp/timestamp.cpp",
        "src/main/native/cpp/SafeThread.cpp",
    ],
    hdrs = glob(
        [
            "src/main/native/include/**",
        ],
        exclude = ["STLExtras.h"],
    ),
    copts = [
        "-Wno-unused-parameter",
    ],
    includes = [
        "src/main/native/include",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
)
