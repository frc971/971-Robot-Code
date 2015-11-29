package(default_visibility = ['//visibility:public'])

load('/tools/build_rules/select', 'cpu_select')

[ cc_library(
  name = libname,
  deps = cpu_select({
    "roborio": ['@arm-frc-linux-gnueabi-repo//:' + libname],
    "amd64": ['@usr_repo//:' + libname],
  }),
) for libname in ['libpthread', 'libm', 'libdl', 'librt']]
