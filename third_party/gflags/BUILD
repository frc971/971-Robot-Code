licenses(['notice'])

common_copts = [
  '-Ithird_party/gflags/include/gflags',
  '-Ithird_party/empty_config_h',

  '-DHAVE_STDINT_H=',
  '-DHAVE_SYS_TYPES_H=',
  '-DHAVE_INTTYPES_H=',
  '-DHAVE_SYS_STAT_H=',
  '-DHAVE_UNISTD_H=',
  '-DHAVE_FNMATCH_H=',
  '-DHAVE_STRTOLL=',
  '-DHAVE_PTHREAD=',
  '-DHAVE_RWLOCK=',
  '-D__STDC_FORMAT_MACROS=1',
  '-DPATH_SEPARATOR=\"\'/\'\"',
  '-DGFLAGS_IS_A_DLL=0',
  '-DGFLAGS_DLL_DECL=',
  '-DGFLAGS_DLL_DEFINE_FLAG=',

  '-Wno-format-nonliteral',
  '-Wno-unused-local-typedefs',
  '-Wno-sign-compare',
  '-Wno-missing-field-initializers',
]

cc_library(
  name = 'gflags',
  visibility = ['//visibility:public'],
  srcs = glob([
    'src/*.cc',
    'src/*.h',
  ], exclude=[
    'src/windows_*',
  ]) + [
    'include/gflags/gflags_declare.h',
    'include/gflags/gflags_gflags.h',
  ],
  hdrs = [
    'include/gflags/gflags.h',
    'include/gflags/gflags_completions.h',
  ],
  deps = [
    '//debian:libpthread',
    '//third_party/empty_config_h',
  ],
  copts = common_copts,
  includes = [ 'include' ],
)
