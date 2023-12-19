filegroup(
    name = "sysroot_files",
    srcs = glob(
        # TODO(austin): Only include the base files here.  Need to figure out what those are.
        # TODO(austin): Generate that list when building the rootfs?
        include = [
            "include/**",
            "lib/**",
            "lib64/**",
            "usr/include/**",
            "usr/lib/**",
            "usr/bin/**",
            "usr/lib64/**",
        ],
        exclude = [
            "usr/share/**",
            "usr/bin/X11",
        ],
    ),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "nppi",
    srcs = [
        "usr/lib/x86_64-linux-gnu/libnppc.so.11",
        "usr/lib/x86_64-linux-gnu/libnppif.so.11",
    ],
    hdrs = glob(
        include = ["usr/include/nppi*.h"],
    ),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "cudart",
    srcs = [
        "usr/lib/x86_64-linux-gnu/libcuda.so.1",
        "usr/lib/x86_64-linux-gnu/libcudart.so.11.0",
    ],
    visibility = ["//visibility:public"],
)

# TODO(austin): lzma, gstreamer, opencv
