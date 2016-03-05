package(default_visibility = ['//visibility:public'])

load('/tools/build_rules/select', 'cpu_select')

[ cc_library(
  name = libname,
  deps = cpu_select({
    "roborio": ['@arm_frc_linux_gnueabi_repo//:' + libname],
    "amd64": ['@usr_repo//:' + libname],
    "armhf": ['@linaro_linux_gcc_4_9_repo//:' + libname],
  }),
) for libname in ['libpthread', 'libm', 'libdl', 'librt']]
