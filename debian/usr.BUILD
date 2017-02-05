load('/tools/build_rules/gtk_dependent', 'gtk_dependent_cc_binary', 'gtk_dependent_cc_library')

package(default_visibility = ['@//debian:__pkg__'])

cc_library(
  name = 'librt',
  srcs = [
    'lib/x86_64-linux-gnu/librt.so'
  ],
)

cc_library(
  name = 'libdl',
  srcs = [
    'lib/x86_64-linux-gnu/libdl.so'
  ],
)

cc_library(
  name = 'libm',
  srcs = [
    'lib/x86_64-linux-gnu/libm.so'
  ],
)

cc_library(
  name = 'libpthread',
)

cc_library(
  name = 'python3.4_lib',
  hdrs = glob(['include/python3.4m/**/*.h']),
  includes = [
    'include/python3.4m/',
  ],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'python3.4_f2py',
  srcs = [
    'lib/python3/dist-packages/numpy/f2py/src/fortranobject.c',
  ],
  hdrs = [
    'lib/python3/dist-packages/numpy/f2py/src/fortranobject.h',
  ],
  copts = [
    '-Wno-error',
    '-Wno-parentheses-equality',
  ],
  includes = [
    'lib/python3/dist-packages/numpy/f2py/src/',
  ],
  deps = [
    ':python3.4_lib',
  ],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'python2.7_lib',
  hdrs = glob(['include/python2.7/**/*.h']),
  srcs = [
    'lib/x86_64-linux-gnu/libpython2.7.so',
  ],
  includes = [
    'include/python2.7/',
  ],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'python2.7_f2py',
  srcs = [
    'lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.c',
  ],
  hdrs = [
    'lib/python2.7/dist-packages/numpy/f2py/src/fortranobject.h',
  ],
  copts = [
    '-Wno-error',
  ],
  includes = [
    'lib/python2.7/dist-packages/numpy/f2py/src/',
  ],
  deps = [
    ':python2.7_lib',
  ],
  visibility = ['//visibility:public'],
)

gtk_dependent_cc_library(
  name = 'gtk+-3.0',
  hdrs = glob([
    'include/gtk-3.0/**/*.h',
    'include/at-spi2-atk/2.0/**/*.h',
    'include/at-spi-2.0/**/*.h',
    'include/dbus-1.0/**/*.h',
    'lib/x86_64-linux-gnu/dbus-1.0/include/**/*.h',
    'include/gtk-3.0/**/*.h',
    'include/gio-unix-2.0/**/*.h',
    'include/cairo/**/*.h',
    'include/pango-1.0/**/*.h',
    'include/harfbuzz/**/*.h',
    'include/pango-1.0/**/*.h',
    'include/atk-1.0/**/*.h',
    'include/pixman-1/**/*.h',
    'include/freetype2/**/*.h',
    'include/libpng12/**/*.h',
    'include/gdk-pixbuf-2.0/**/*.h',
    'include/glib-2.0/**/*.h',
    'lib/x86_64-linux-gnu/glib-2.0/include/**/*.h',
  ]),
  includes = [
    'include/gtk-3.0',
    'include/at-spi2-atk/2.0',
    'include/at-spi-2.0',
    'include/dbus-1.0',
    'lib/x86_64-linux-gnu/dbus-1.0/include',
    'include/gtk-3.0',
    'include/gio-unix-2.0/',
    'include/cairo',
    'include/pango-1.0',
    'include/harfbuzz',
    'include/pango-1.0',
    'include/atk-1.0',
    'include/pixman-1',
    'include/freetype2',
    'include/libpng12',
    'include/gdk-pixbuf-2.0',
    'include/glib-2.0',
    'lib/x86_64-linux-gnu/glib-2.0/include',
  ],
  linkopts = [
    '-lgtk-3',
    '-lgdk-3',
    '-lpangocairo-1.0',
    '-lpango-1.0',
    '-latk-1.0',
    '-lcairo-gobject',
    '-lcairo',
    '-lgdk_pixbuf-2.0',
    '-lgio-2.0',
    '-lgobject-2.0',
    '-lglib-2.0'
  ],
  visibility = ['//visibility:public'],
)
