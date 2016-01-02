licenses(['notice'])

load('/tools/build_rules/select', 'cpu_select', 'compiler_select')
load('/tools/build_rules/empty_main', 'empty_main_if_asan')

common_copts = [
  # Stuff from their Makefile.
  '-Wno-sign-compare',
  '-fno-builtin-malloc',
  '-fno-builtin-free',
  '-fno-builtin-realloc',
  '-fno-builtin-calloc',
  '-fno-builtin-cfree',
  '-fno-builtin-memalign',
  '-fno-builtin-posix_memalign',
  '-fno-builtin-valloc',
  '-fno-builtin-pvalloc',
  '-Wno-unused-result',
  '-fno-omit-frame-pointer',
  '-DNDEBUG',

  # Stuff to make it work for us.
  '-Ithird_party/gperftools/src/',
  '-Ithird_party/empty_config_h',
  '-Wno-unused-parameter',
  '-Wno-missing-field-initializers',
  '-Wno-unused-function',
  '-Wno-unused-variable',
  '-Wno-format-nonliteral',
  '-Wno-switch-enum',
  '-Wno-error=cast-align',
  '-Wno-error=cast-qual',

  # Stuff pulled out of config.h.
  '-DHAVE_BUILTIN_EXPECT=1',
  '-DHAVE_DECL_CFREE=1',
  '-DHAVE_DECL_MEMALIGN=1',
  '-DHAVE_DECL_POSIX_MEMALIGN=1',
  '-DHAVE_DECL_PVALLOC=1',
  '-DHAVE_DECL_UNAME=1',
  '-DHAVE_DECL_VALLOC=1',
  '-DHAVE_DLFCN_H=1',
  '-DHAVE_ELF32_VERSYM=1',
  '-DHAVE_EXECINFO_H=1',
  '-DHAVE_FCNTL_H=1',
  '-DHAVE_FEATURES_H=1',
  '-DHAVE_FORK=1',
  '-DHAVE_GETEUID=1',
  '-DHAVE_GLOB_H=1',
  '-DHAVE_GRP_H=1',
  '-DHAVE_INTTYPES_H=1',
  '-DHAVE_LINUX_PTRACE_H=1',
  '-DHAVE_LINUX_SIGEV_THREAD_ID=1',
  '-DHAVE_MALLOC_H=1',
  '-DHAVE_MEMORY_H=1',
  '-DHAVE_MMAP=1',
  '-DHAVE_NAMESPACES=1',
  '-DHAVE_POLL_H=1',
  '-DHAVE_PROGRAM_INVOCATION_NAME=1',
  '-DHAVE_PTHREAD=1',
  '-DHAVE_PWD_H=1',
  '-DHAVE_SBRK=1',
  '-DHAVE_SCHED_H=1',
  '-DHAVE_STDINT_H=1',
  '-DHAVE_STDLIB_H=1',
  '-DHAVE_STRINGS_H=1',
  '-DHAVE_STRING_H=1',
  '-DHAVE_STRUCT_MALLINFO=1',
  '-DHAVE_SYS_CDEFS_H=1',
  '-DHAVE_SYS_PRCTL_H=1',
  '-DHAVE_SYS_RESOURCE_H=1',
  '-DHAVE_SYS_SOCKET_H=1',
  '-DHAVE_SYS_STAT_H=1',
  '-DHAVE_SYS_SYSCALL_H=1',
  '-DHAVE_SYS_TYPES_H=1',
  '-DHAVE_SYS_UCONTEXT_H=1',
  '-DHAVE_SYS_WAIT_H=1',
  '-DHAVE_TLS=1',
  '-DHAVE_UCONTEXT_H=1',
  '-DHAVE_UNISTD_H=1',
  '-DHAVE_UNWIND_H=1',
  '-DHAVE___ATTRIBUTE__=1',
  '-DHAVE___ENVIRON=1',
  '-DMALLOC_HOOK_MAYBE_VOLATILE=volatile',
  '-DPERFTOOLS_DLL_DECL=',
  '-DSTDC_HEADERS=1',
  '-DSTL_NAMESPACE=std',
  '-DPACKAGE_STRING=\\"gperftools\\ 2.4\\"',
  '-DPACKAGE_BUGREPORT=\\"http://frc971.org/contact\\"',
  '-DPACKAGE_VERSION=\\"2.4\\"',
] + cpu_select({
  'amd64': [
    '-DHAVE_GETPAGESZE=1',
    '-DHAVE_SYS_PARAM_H=1',
    '-DPC_FROM_UCONTEXT=uc_mcontext.gregs[REG_RIP]',
    '-DPRIdS=\\"ld\\"',
    '-DPRIuS=\\"lu\\"',
    '-DPRIxS=\\"lx\\"',
  ],
  'roborio': [
    '-DPC_FROM_UCONTEXT=uc_mcontext.arm_pc',
    '-DPRIdS=\\"d\\"',
    '-DPRIuS=\\"u\\"',
    '-DPRIxS=\\"x\\"',
  ],
}) + compiler_select({
  'clang': [
    '-Wno-unused-const-variable',
    '-Wno-gnu-alignof-expression',
    '-Wno-unused-private-field',
  ],
  'gcc': [],
})

cc_library(
  name = 'tcmalloc',
  visibility = ['//visibility:public'],
  hdrs = glob([
    'src/*.h',
    'src/base/*.h',
    'src/gperftools/*.h',
  ]) + [
    'src/third_party/valgrind.h',
  ],
  srcs = glob(include = [
    'src/*.cc',
    'src/*.c',
    'src/base/*.cc',
    'src/base/*.c',
  ], exclude = [
    '**/*_unittest.cc',
    '**/*_test.cc',
    'src/debugallocation.cc',
  ]),
  deps = [
    '//debian:librt',
    '//third_party/empty_config_h',
  ],
  copts = common_copts,
  alwayslink = True,
  nocopts = '-std=gnu\+\+1y',
)

cc_library(
  name = 'testutil',
  srcs = [
    'src/tests/testutil.cc',
  ],
  hdrs = [
    'src/tests/testutil.h',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
)

cc_test(
  name = 'low_level_alloc_unittest',
  srcs = [
    'src/tests/low_level_alloc_unittest.cc',
  ],
  defines = [
    'NO_TCMALLOC_SAMPLES',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'medium',
)

cc_test(
  name = 'atomicops_unittest',
  srcs = [
    'src/tests/atomicops_unittest.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'stacktrace_unittest',
  srcs = [
    'src/tests/stacktrace_unittest.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'tcmalloc_unittest',
  srcs = empty_main_if_asan([
    'src/tests/tcmalloc_unittest.cc',
  ]),
  deps = [
    ':tcmalloc',
    ':testutil',
  ],
  copts = common_copts + [
    '-fno-builtin',
  ],
  size = 'small',
)

cc_test(
  name = 'tcmalloc_large_unittest',
  srcs = empty_main_if_asan([
    'src/tests/tcmalloc_large_unittest.cc',
  ]),
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts + [
    '-fno-builtin',
  ],
  size = 'small',
)

cc_test(
  name = 'addressmap_unittest',
  srcs = [
    'src/tests/addressmap_unittest.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'medium',
)

cc_test(
  name = 'system_alloc_unittest',
  srcs = empty_main_if_asan([
    'src/tests/system-alloc_unittest.cc',
  ]),
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts + [
    '-fno-builtin',
  ],
  size = 'small',
)

cc_test(
  name = 'packed_cache_test',
  srcs = [
    'src/tests/packed-cache_test.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'frag_unittest',
  srcs = [
    'src/tests/frag_unittest.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'markidle_unittest',
  srcs = empty_main_if_asan([
    'src/tests/markidle_unittest.cc',
  ]),
  deps = [
    ':tcmalloc',
    ':testutil',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'current_allocated_bytes_test',
  srcs = [
    'src/tests/current_allocated_bytes_test.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'malloc_hook_test',
  srcs = [
    'src/tests/malloc_hook_test.cc',
  ],
  deps = [
    ':tcmalloc',
    ':testutil',
  ],
  copts = common_copts + compiler_select({
    'gcc': [
      '-Wno-maybe-uninitialized',
    ],
    'clang': [],
  }),
  size = 'small',
)

cc_test(
  name = 'malloc_extension_test',
  srcs = empty_main_if_asan([
    'src/tests/malloc_extension_test.cc',
  ]),
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'malloc_extension_c_test',
  srcs = empty_main_if_asan([
    'src/tests/malloc_extension_c_test.c',
  ]),
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'memalign_unittest',
  srcs = empty_main_if_asan([
    'src/tests/memalign_unittest.cc',
  ]),
  deps = [
    ':tcmalloc',
    ':testutil',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'page_heap_test',
  srcs = [
    'src/tests/page_heap_test.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'pagemap_unittest',
  srcs = empty_main_if_asan([
    'src/tests/pagemap_unittest.cc',
  ]),
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'realloc_unittest',
  srcs = [
    'src/tests/realloc_unittest.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'stack_trace_table_test',
  srcs = [
    'src/tests/stack_trace_table_test.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'thread_dealloc_unittest',
  srcs = [
    'src/tests/thread_dealloc_unittest.cc',
  ],
  deps = [
    ':tcmalloc',
    ':testutil',
  ],
  copts = common_copts,
  size = 'small',
)

'''
We don't build this because it actually needs to be in a separate binary.
cc_test(
  name = 'debugallocation_test',
  srcs = [
    'src/tests/debugallocation_test.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)
'''

cc_test(
  name = 'tcmalloc_large_heap_fragmentation_unittest',
  srcs = empty_main_if_asan([
    'src/tests/large_heap_fragmentation_unittest.cc',
  ]),
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'raw_printer_test',
  srcs = [
    'src/tests/raw_printer_test.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'getpc_test',
  srcs = empty_main_if_asan([
    'src/tests/getpc_test.cc',
  ]),
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'profiledata_unittest',
  srcs = [
    'src/tests/profiledata_unittest.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'profile_handler_unittest',
  srcs = [
    'src/tests/profile-handler_unittest.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  flaky = True,
  size = 'small',
)

cc_test(
  name = 'heap_profiler_unittest',
  srcs = [
    'src/tests/heap-profiler_unittest.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts,
  size = 'small',
)

cc_test(
  name = 'sampler_test',
  srcs = [
    'src/tests/sampler_test.cc',
  ],
  deps = [
    ':tcmalloc',
  ],
  copts = common_copts + [
    '-Wno-type-limits',
  ],
  size = 'small',
)
