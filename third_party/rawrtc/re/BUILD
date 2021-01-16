load("@//tools/build_rules:select.bzl", "compiler_select")

cc_library(
    name = "re",
    srcs = glob(
        [
            "src/**/*.c",
            "src/**/*.h",
        ],
        exclude = [
            "src/aes/apple/**",
            "src/aes/stub.c",
            "src/hmac/apple/**",
            "src/hmac/hmac.c",
            "src/mqueue/win32/**",
            "src/tls/openssl/**",
            "src/dns/win32/**",
            "src/mod/win32/**",
            "src/lock/lock.c",
            "src/lock/win32/**",
            "src/dns/darwin/**",
            "src/net/win32/**",
            "src/net/bsd/**",
        ],
    ),
    hdrs = glob(["include/*.h"]),
    copts = compiler_select({
        "clang": [
            "-Wno-incompatible-pointer-types-discards-qualifiers",
        ],
        "gcc": [
            "-Wno-discarded-qualifiers",
            "-Wno-implicit-fallthrough",
        ],
    }) + [
        "-DVER_MAJOR=0",
        "-DVER_MINOR=6",
        "-DVER_PATCH=0",
        "-DHAVE_ROUTE_LIST",
        "-Wno-cast-qual",
        "-Wno-cast-align",
        "-Wno-implicit-function-declaration",
    ],
    defines = ["HAVE_INTTYPES_H"],
    includes = ["include/"],
    visibility = ["//visibility:public"],
    deps = ["@boringssl//:ssl"],
)
