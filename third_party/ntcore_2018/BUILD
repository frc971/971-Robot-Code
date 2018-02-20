licenses(["notice"])

cc_library(
    name = "ntcore",
    srcs = glob(
        [
            "src/**/*.cpp",
            "src/**/*.h",
            "wpiutil/src/**/*.cpp",
        ],
        exclude = [
            "src/main/native/cpp/networktables/**",
            "src/test/**",
            "src/main/native/cpp/jni/**",
        ],
    ),
    copts = [
        "-Wno-unused-parameter",
    ],
    includes = [
        "src/main/native/include",
    ],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/wpiutil_2018:wpiutil",
    ],
)
