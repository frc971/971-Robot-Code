{
  'targets': [
    {
      'target_name': 'libgcc-4.5.2',
      'type': 'static_library',
      'variables': {
        'compiler-rt': '<(externals)/compiler-rt-<(compiler_rt_version)',
      },
      'include_dirs': [
        '<(compiler-rt)/lib',
      ],
      'defines': [
        '_YUGA_BIG_ENDIAN=1',
        '_YUGA_LITTLE_ENDIAN=0',
        'UINT64_C(c)=c##ULL',
      ],
      'sources': [
        '<(compiler-rt)/lib/powisf2.c',
        '<(compiler-rt)/lib/powidf2.c',
        '<(compiler-rt)/lib/mulsc3.c',
        '<(compiler-rt)/lib/muldc3.c',
        '<(compiler-rt)/lib/divsc3.c',
        '<(compiler-rt)/lib/divdc3.c',
        #'<(compiler-rt)/lib/bswapsi2.c',
        '_bswapsi2.o',
        #'<(compiler-rt)/lib/bswapdi2.c',
        '_bswapdi2.o',
        '<(compiler-rt)/lib/floatundisf.c',
        '<(compiler-rt)/lib/floatundidf.c',

        'libm.c',
      ],
    },
  ],
}
