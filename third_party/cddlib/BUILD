licenses(["notice"])

load("//tools/build_rules:select.bzl", "compiler_select")

cc_library(
    name = "cddlib",
    srcs = [
        "lib-src/cddcore.c",
        "lib-src/cddio.c",
        "lib-src/cddlib.c",
        "lib-src/cddlp.c",
        "lib-src/cddmp.c",
        "lib-src/cddmp.h",
        "lib-src/cddproj.c",
        "lib-src/cddtypes.h",
        "lib-src/setoper.c",
        "lib-src/setoper.h",
    ],
    hdrs = [
        "lib-src/cdd.h",
    ],
    copts = [
        "-Wno-sometimes-uninitialized",
        "-Wno-unused-parameter",
        "-Wno-switch-enum",
        "-Wno-empty-body",
        "-Wno-sign-compare",
        "-Wno-implicit-fallthrough",
        "-Wno-unused-result",
    ] + compiler_select({
        "gcc": ["-Wno-unused-but-set-variable"],
        "clang": [],
    }),
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "_cddlib.so",
    linkshared = True,
    linkstatic = False,
    visibility = ["//visibility:public"],
    deps = [":cddlib"],
)
