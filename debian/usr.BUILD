load("@//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary", "gtk_dependent_cc_library")

package(default_visibility = ["@//debian:__pkg__"])

cc_library(
    name = "librt",
    srcs = [
        "lib/x86_64-linux-gnu/librt.so",
    ],
)

cc_library(
    name = "libdl",
    srcs = [
        "lib/x86_64-linux-gnu/libdl.so",
    ],
)

cc_library(
    name = "libm",
    srcs = [
        "lib/x86_64-linux-gnu/libm.so",
    ],
)

cc_library(
    name = "libpthread",
)

cc_library(
    name = "python3.4_lib",
    hdrs = glob(["include/python3.4m/**/*.h"]),
    includes = [
        "include/python3.4m/",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "python3.4_f2py",
    srcs = [
        "lib/python3/dist-packages/numpy/f2py/src/fortranobject.c",
    ],
    hdrs = [
        "lib/python3/dist-packages/numpy/f2py/src/fortranobject.h",
    ],
    copts = [
        "-Wno-error",
        "-Wno-parentheses-equality",
    ],
    includes = [
        "lib/python3/dist-packages/numpy/f2py/src/",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":python3.4_lib",
    ],
)

cc_library(
    name = "python2.7_lib",
    srcs = [
        "lib/x86_64-linux-gnu/libpython2.7.so",
    ],
    hdrs = glob(["include/python2.7/**/*.h"]),
    includes = [
        "include/python2.7/",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "python2.7_f2py",
    srcs = [
        "lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.c",
    ],
    hdrs = [
        "lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.h",
    ],
    copts = [
        "-Wno-error",
    ],
    includes = [
        "lib/python2.7/dist-packages/numpy/f2py/src/",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":python2.7_lib",
    ],
)

gtk_dependent_cc_library(
    name = "gtk+-3.0",
    srcs = [
        "lib/x86_64-linux-gnu/libatk-1.0.so",
        "lib/x86_64-linux-gnu/libcairo.so",
        "lib/x86_64-linux-gnu/libcairo-gobject.so",
        "lib/x86_64-linux-gnu/libgdk-3.so",
        "lib/x86_64-linux-gnu/libgdk_pixbuf-2.0.so",
        "lib/x86_64-linux-gnu/libgio-2.0.so",
        "lib/x86_64-linux-gnu/libglib-2.0.so",
        "lib/x86_64-linux-gnu/libgobject-2.0.so",
        "lib/x86_64-linux-gnu/libgtk-3.so",
        "lib/x86_64-linux-gnu/libpango-1.0.so",
        "lib/x86_64-linux-gnu/libpangocairo-1.0.so",
    ],
    hdrs = glob([
        "include/gtk-3.0/**/*.h",
        "include/at-spi2-atk/2.0/**/*.h",
        "include/at-spi-2.0/**/*.h",
        "include/dbus-1.0/**/*.h",
        "lib/x86_64-linux-gnu/dbus-1.0/include/**/*.h",
        "include/gtk-3.0/**/*.h",
        "include/gio-unix-2.0/**/*.h",
        "include/cairo/**/*.h",
        "include/pango-1.0/**/*.h",
        "include/harfbuzz/**/*.h",
        "include/pango-1.0/**/*.h",
        "include/atk-1.0/**/*.h",
        "include/pixman-1/**/*.h",
        "include/freetype2/**/*.h",
        "include/libpng12/**/*.h",
        "include/gdk-pixbuf-2.0/**/*.h",
        "include/glib-2.0/**/*.h",
        "lib/x86_64-linux-gnu/glib-2.0/include/**/*.h",
    ]),
    includes = [
        "include/at-spi-2.0",
        "include/at-spi2-atk/2.0",
        "include/atk-1.0",
        "include/cairo",
        "include/dbus-1.0",
        "include/freetype2",
        "include/gdk-pixbuf-2.0",
        "include/gio-unix-2.0/",
        "include/glib-2.0",
        "include/gtk-3.0",
        "include/harfbuzz",
        "include/libpng12",
        "include/pango-1.0",
        "include/pixman-1",
        "lib/x86_64-linux-gnu/dbus-1.0/include",
        "lib/x86_64-linux-gnu/glib-2.0/include",
    ],
    visibility = ["//visibility:public"],
)
