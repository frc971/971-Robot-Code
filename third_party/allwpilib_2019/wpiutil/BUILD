licenses(["notice"])

cc_library(
    name = "wpiutil",
    srcs = glob([
        "src/main/native/cpp/llvm/*.cpp",
    ]) + [
        "src/main/native/cpp/llvm/Unix/Path.inc",
        "src/main/native/cpp/timestamp.cpp",
        "src/main/native/cpp/SafeThread.cpp",
        "src/main/native/cpp/memory.cpp",
    ],
    hdrs = glob([
        "src/main/native/include/**",
    ]),
    copts = [
        "-Wno-unused-parameter",
    ],
    includes = [
        "src/main/native/include",
    ],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
)
