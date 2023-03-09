load("@rules_pkg//:pkg.bzl", "pkg_tar")
load(":files.bzl", "LIB_SYMLINKS", "LIBS")

pkg_tar(
    name = "runtime",
    srcs = LIBS + [
        "bin/julia",
    ] + glob([
        "share/julia/**/*.jl",
        "share/julia/**/*.toml",
        "include/julia/**/*",
    ], exclude = [
        "**/test/**",
    ]),
    symlinks = LIB_SYMLINKS,
    strip_prefix = "external/julia",
    visibility = ["//visibility:public"],
)
