licenses(['notice'])

load('/tools/build_rules/select', 'compiler_select')

cc_library(
  name = 'cddlib',
  visibility = ['//visibility:public'],
  srcs = [
    'lib-src/cddcore.c',
    'lib-src/cddlp.c',
    'lib-src/cddmp.c',
    'lib-src/cddio.c',
    'lib-src/cddlib.c',
    'lib-src/cddproj.c',
    'lib-src/setoper.c',
    'lib-src/cddmp.h',
    'lib-src/cddtypes.h',
    'lib-src/setoper.h',
  ],
  hdrs = [
    'lib-src/cdd.h',
  ],
  copts = [
    '-Wno-sometimes-uninitialized',
    '-Wno-unused-parameter',
    '-Wno-switch-enum',
    '-Wno-empty-body',
    '-Wno-sign-compare',
    '-Wno-unused-result',
  ] + compiler_select({
      'gcc': ['-Wno-unused-but-set-variable'],
      'clang': []
  }),
)
cc_binary(
  name = '_cddlib.so',
  deps = [':cddlib'],
  linkshared = True,
  visibility = ['//visibility:public'],
  linkstatic = False,
)
