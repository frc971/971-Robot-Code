filegroup(
    name = "sysroot_files",
    srcs = glob(
        include = [
            "include/**",
            "lib/**",
            "lib64/**",
            "usr/include/**",
            "usr/lib/**",
            "usr/lib64/**",
        ],
        exclude = [
            "usr/share/**",
        ],
    ),
    visibility = ["//visibility:public"],
)
