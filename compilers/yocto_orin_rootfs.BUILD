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

cc_library(
    name = "argus",
    srcs = [
        "usr/lib/libnvargus_socketclient.so",
    ],
    hdrs = glob(
        include = ["usr/include/Argus/**"],
    ),
    includes = ["usr/include/Argus/utils/"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "eglstream",
    srcs = [
        #"usr/lib/libnvargus_socketclient.so",
    ],
    hdrs = glob(
        include = ["usr/include/EGLStream/**"],
    ),
    includes = ["usr/include/EGLStream/"],
    visibility = ["//visibility:public"],
)
