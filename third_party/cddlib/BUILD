licenses(["notice"])

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
        "-Wno-unused-but-set-variable",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "_cddlib.so",
    linkshared = True,
    linkstatic = False,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [":cddlib"],
)
