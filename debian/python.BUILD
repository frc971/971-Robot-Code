package(default_visibility = ["@//debian:__pkg__"])

cc_library(
    name = "python3.5_lib",
    srcs = [
        "usr/lib/x86_64-linux-gnu/libpython3.5m.so",
    ],
    hdrs = glob(["usr/include/**/*.h"]),
    includes = [
        "usr/include/",
        "usr/include/python3.5m/",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "python3.5_f2py",
    srcs = [
        "usr/lib/python3/dist-packages/numpy/f2py/src/fortranobject.c",
    ],
    hdrs = [
        "usr/lib/python3/dist-packages/numpy/f2py/src/fortranobject.h",
    ],
    copts = [
        "-Wno-error",
        "-Wno-parentheses-equality",
    ],
    includes = [
        "usr/lib/python3/dist-packages/numpy/f2py/src/",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":python3.5_lib",
    ],
)

cc_library(
    name = "python2.7_lib",
    srcs = [
        "usr/lib/x86_64-linux-gnu/libpython2.7.so",
    ],
    hdrs = glob([
        "usr/include/**/*.h",
    ]),
    includes = [
        "usr/include/",
        "usr/include/python2.7/",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "python2.7_f2py",
    srcs = [
        "usr/lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.c",
    ],
    hdrs = [
        "usr/lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.h",
    ],
    copts = [
        "-Wno-error",
    ],
    includes = [
        "usr/lib/python2.7/dist-packages/numpy/f2py/src/",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":python2.7_lib",
    ],
)

filegroup(
    name = "all_files",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)

genrule(
    name = "copy_f2py",
    srcs = ["usr/bin/f2py"],
    outs = ["f2py.py"],
    cmd = "cp $< $@",
    executable = True,
)

py_binary(
    name = "f2py",
    srcs = ["f2py.py"],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "scipy",
    srcs = glob([
        "usr/lib/python3/dist-packages/numpy",
        "usr/lib/python3/dist-packages/scipy",
        "usr/lib/python2.7/dist-packages/numpy",
        "usr/lib/python2.7/dist-packages/scipy",
    ]),
    visibility = ["//visibility:public"],
)
