licenses(['notice'])

common_copts = [
  '-Ithird_party/empty_config_h',

  '-DGOOGLE_NAMESPACE=::ctemplate',
  '-DHASH_MAP_H=<tr1/unordered_map>',
  '-DHASH_NAMESPACE=std::tr1',
  '-DHASH_SET_H=<tr1/unordered_set>',
  '-DHAVE_BYTESWAP_H=1',
  '-DHAVE_DIRENT_H=1',
  '-DHAVE_DLFCN_H=1',
  '-DHAVE_ENDIAN_H=1',
  '-DHAVE_GETOPT=1',
  '-DHAVE_GETOPT_H=1',
  '-DHAVE_GETOPT_LONG=1',
  '-DHAVE_HASH_MAP=1',
  '-DHAVE_HASH_SET=1',
  '-DHAVE_INTTYPES_H=1',
  '-DHAVE_MEMORY_H=1',
  '-DHAVE_NAMESPACES=1',
  '-DHAVE_PTHREAD=1',
  '-DHAVE_RWLOCK=1',
  '-DHAVE_STDINT_H=1',
  '-DHAVE_STDLIB_H=1',
  '-DHAVE_STRINGS_H=1',
  '-DHAVE_STRING_H=1',
  '-DHAVE_SYS_STAT_H=1',
  '-DHAVE_SYS_TYPES_H=1',
  '-DHAVE_UINT32_T=1',
  '-DHAVE_UINT64_T=1',
  '-DHAVE_UNISTD_H=1',
  '-DHAVE_UNORDERED_MAP=1',
  '-DHAVE_UTIME_H=1',
  '-DHAVE_U_INT32_T=1',
  '-DHAVE_U_INT64_T=1',
  '-DHAVE___ATTRIBUTE__=1',
  '-DHTMLPARSER_NAMESPACE=ctemplate_htmlparser',
  '-DINTERLOCKED_EXCHANGE_NONVOLATILE=1',
  '-DPRIdS="ld"',
  '-DPRIuS="lu"',
  '-DPRIxS="lx"',
  '-DSTDC_HEADERS=1',
  '-DSTL_NAMESPACE=std',
  '-D_END_GOOGLE_NAMESPACE_=}',
  '-D_START_GOOGLE_NAMESPACE_=namespace\\ ctemplate\\ {',
]

cc_library(
  name = 'ctemplate',
  visibility = ['//visibility:public'],
  includes = ['src'],
  srcs = glob([
    'src/base/*.cc',
    'src/base/*.h',
    'src/htmlparser/*.cc',
    'src/htmlparser/*.h',
  ]) + [
    ':htmlparser_fsm',
    ':jsparser_fsm',
  ],
  hdrs = glob([
    'src/ctemplate/*.h',
  ]),
  copts = common_copts,
)

py_binary(
  name = 'generate_fsm',
  srcs = [
    'src/htmlparser/generate_fsm.py',
    'src/htmlparser/fsm_config.py',
  ],
)

genrule(
  name = 'htmlparser_fsm',
  tools = [
    ':generate_fsm',
  ],
  srcs = [
    'src/htmlparser/htmlparser_fsm.config',
  ],
  outs = [
    'src/htmlparser/htmlparser_fsm.h',
  ],
  cmd = '$(location :generate_fsm) $< > $@',
)

genrule(
  name = 'jsparser_fsm',
  tools = [
    ':generate_fsm',
  ],
  srcs = [
    'src/htmlparser/jsparser_fsm.config',
  ],
  outs = [
    'src/htmlparser/jsparser_fsm.h',
  ],
  cmd = '$(location :generate_fsm) $< > $@',
)
