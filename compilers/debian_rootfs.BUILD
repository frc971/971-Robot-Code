filegroup(
    name = "sysroot_files",
    srcs = glob(
        include = [
            "include/**",
            "lib/**",
            "usr/include/**",
            "usr/lib/**",
        ],
        exclude = [
            "usr/share/**",
        ],
    ),
    visibility = ["//visibility:public"],
)
