licenses(["notice"])

cc_library(
    name = "seasocks",
    srcs = glob(
        include = ["src/main/c/**/*.cpp"],
        exclude = ["src/main/c/seasocks/ZlibContext.cpp"],
    ),
    hdrs = glob(
        include = [
            "src/main/c/**/*.h",
        ],
        exclude = [
            "src/main/internal/**/*",
        ],
    ),
    copts = [
        # TODO(Brian): Don't apply this to all of the code...
        "-Wno-cast-align",
        "-Wno-cast-qual",
        "-Wno-switch-enum",
        "-Wno-format-nonliteral",
        "-Wno-unused-parameter",
    ],
    includes = ["src/main/c"],
    visibility = ["//visibility:public"],
)
