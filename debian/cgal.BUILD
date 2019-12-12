cc_library(
    name = "cgal",
    srcs = [
        "usr/lib/libCGAL.so.10",
        "usr/lib/x86_64-linux-gnu/libgmp.so.10.2.0",
    ],
    hdrs = glob([
        "usr/include/**/*.h",
        "usr/include/**/*.hpp",
    ]),
    includes = [
        "usr/include",
        "usr/include/x86_64-linux-gnu",
    ],
    visibility = ["//visibility:public"],
)
