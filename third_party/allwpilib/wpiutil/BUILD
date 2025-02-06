licenses(["notice"])

cc_library(
    name = "wpiutil",
    srcs = glob(
        [
            "src/main/native/thirdparty/llvm/cpp/llvm/*.cpp",
            "src/main/native/thirdparty/fmtlib/src/*.cpp",
        ],
    ) + [
        "src/main/native/cpp/timestamp.cpp",
        "src/main/native/cpp/Synchronization.cpp",
        "src/main/native/cpp/fs.cpp",
        "src/main/native/cpp/MappedFileRegion.cpp",
        "src/main/native/cpp/SafeThread.cpp",
        "src/main/native/cpp/string.cpp",
    ],
    hdrs = glob(
        [
            "src/main/native/include/**",
            "src/main/native/thirdparty/fmtlib/include/**",
            "src/main/native/thirdparty/llvm/include/**",
            "src/main/native/thirdparty/expected/include/**/*",
        ],
        exclude = ["STLExtras.h"],
    ),
    copts = [
        "-Wno-unused-parameter",
    ],
    includes = [
        "src/main/native/include",
        "src/main/native/thirdparty/fmtlib/include",
        "src/main/native/thirdparty/llvm/include",
        "src/main/native/thirdparty/expected/include",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = ["@allwpilib_ni_libraries//:ni-libraries"],
)
