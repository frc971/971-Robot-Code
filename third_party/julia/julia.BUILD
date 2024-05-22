load("@rules_pkg//:pkg.bzl", "pkg_tar")
load(":files.bzl", "LIBS", "LIB_SYMLINKS")

pkg_tar(
    name = "runtime",
    srcs = LIBS + [
        "bin/julia",
    ] + glob(
        [
            "share/julia/**/*.jl",
            "share/julia/**/*.toml",
            "include/julia/**/*",
        ],
        exclude = [
            "**/test/**",
        ],
    ),
    strip_prefix = "external/julia",
    symlinks = LIB_SYMLINKS,
    visibility = ["//visibility:public"],
)
