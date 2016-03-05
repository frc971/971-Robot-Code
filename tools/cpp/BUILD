package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'empty_main',
  srcs = [ 'empty_main.c' ],
)

cc_library(
  name = 'malloc',
  deps = select({
    '//tools:has_asan': [],
    '//tools:has_tsan': [],
    '//conditions:default': ['//third_party/gperftools:tcmalloc'],
  }),
)

cc_library(
  name = 'stl',
)

filegroup(
  name = 'empty',
  srcs = [],
)

# This is the entry point for --crosstool_top.  Toolchains are found
# by lopping off the name of --crosstool_top and searching for
# 'cc-compiler-${CPU}' in this BUILD file, where CPU is the target CPU
# specified in --cpu.
#
# This file group should include
#   * all cc_toolchain targets supported
#   * all file groups that said cc_toolchain might refer to,
# including the default_grte_top setting in the CROSSTOOL
# protobuf.
filegroup(
  name = 'toolchain',
  srcs = [
    ':cc-compiler-k8',
    ':cc-compiler-roborio',
    '@arm_frc_linux_gnueabi_repo//:compiler_components',
    ':roborio-compiler-files',
    ':flags_compiler_inputs',
    '@linaro_linux_gcc_4_9_repo//:compiler_components',
    ':linaro-gcc-files',
  ],
)

# Compiler inputs given by --copt etc in //tools:bazel.rc.
filegroup(
  name = 'flags_compiler_inputs',
  srcs = select({
    '//tools:has_asan': [
      'asan-blacklist',
    ],
    '//tools:has_ubsan': [
      'ubsan-blacklist',
    ],
    '//conditions:default': [],
  }),
)

cc_toolchain(
  name = 'cc-compiler-k8',
  all_files = ':flags_compiler_inputs',
  compiler_files = ':flags_compiler_inputs',
  cpu = 'local',
  dwp_files = ':empty',
  dynamic_runtime_libs = [':empty'],
  linker_files = ':empty',
  objcopy_files = ':empty',
  static_runtime_libs = [':empty'],
  strip_files = ':empty',
  supports_param_files = 1,
)

filegroup(
  name = 'roborio-compiler-files',
  srcs = [
    '//tools/cpp/arm-frc-linux-gnueabi:tool-wrappers',
    '//tools/cpp/arm-frc-linux-gnueabi:as',
    '@arm_frc_linux_gnueabi_repo//:compiler_pieces',
    ':flags_compiler_inputs',
  ],
)

filegroup(
  name = 'roborio_linker_files',
  srcs = [
    '//tools/cpp/arm-frc-linux-gnueabi:ld',
    '//tools/cpp/arm-frc-linux-gnueabi:ar',
    '//tools/cpp/arm-frc-linux-gnueabi:gcc',
    '@arm_frc_linux_gnueabi_repo//:compiler_pieces',
  ],
)
filegroup(
  name = 'roborio_compiler_files',
  srcs = [
    '//tools/cpp/arm-frc-linux-gnueabi:gcc',
    '//tools/cpp/arm-frc-linux-gnueabi:ld',
  ],
)

cc_toolchain(
  name = 'cc-compiler-roborio',
  all_files = ':roborio-compiler-files',
  compiler_files = ':roborio_compiler_files',
  cpu = 'roborio',
  dwp_files = ':empty',
  dynamic_runtime_libs = [':empty'],
  linker_files = ':roborio_linker_files',
  objcopy_files = '//tools/cpp/arm-frc-linux-gnueabi:objcopy',
  static_runtime_libs = [':empty'],
  strip_files = '//tools/cpp/arm-frc-linux-gnueabi:strip',
  supports_param_files = 1,
)

filegroup(
  name = 'linaro-gcc-files',
  srcs = [
    '//tools/cpp/linaro_linux_gcc:clang-symlinks',
    '//tools/cpp/linaro_linux_gcc:tool-wrappers',
    '@linaro_linux_gcc_4_9_repo//:compiler_pieces',
  ],
)

filegroup(
  name = 'linaro_linux_linker_files',
  srcs = [
    '//tools/cpp/linaro_linux_gcc:gcc',
    '//tools/cpp/linaro_linux_gcc:ld',
    '//tools/cpp/linaro_linux_gcc:ar',
    '//tools/cpp/linaro_linux_gcc:clang-ld',
    '//tools/cpp/linaro_linux_gcc:clang',
    '@linaro_linux_gcc_4_9_repo//:compiler_pieces',
  ],
)

filegroup(
  name = 'linaro_linux_compiler_files',
  srcs = [
    '//tools/cpp/linaro_linux_gcc:gcc',
    '//tools/cpp/linaro_linux_gcc:ld',
    '//tools/cpp/linaro_linux_gcc:clang',
    '//tools/cpp/linaro_linux_gcc:as',
  ],
)

cc_toolchain(
  name = 'cc-compiler-armhf-debian',
  all_files = ':linaro-gcc-files',
  compiler_files = ':linaro_linux_compiler_files',
  cpu = 'armhf',
  dwp_files = ':empty',
  dynamic_runtime_libs = [':empty'],
  linker_files = ':linaro_linux_linker_files',
  objcopy_files = '//tools/cpp/linaro_linux_gcc:objcopy',
  static_runtime_libs = [':empty'],
  strip_files = '//tools/cpp/linaro_linux_gcc:strip',
  supports_param_files = 1,
)
