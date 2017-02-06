workspace(name = 'org_frc971')

new_local_repository(
  name = 'usr_repo',
  path = '/usr',
  build_file = 'debian/usr.BUILD',
)

new_git_repository(
  name = 'slycot_repo',
  remote = 'https://github.com/avventi/Slycot.git',
  build_file = 'debian/slycot.BUILD',
  commit = '5af5f283cb23cbe23c4dfea4d5e56071bdbd6e70',
)

bind(
  name = 'slycot',
  actual = '@slycot_repo//:slycot',
)

new_http_archive(
  name = 'arm_frc_linux_gnueabi_repo',
  build_file = 'tools/cpp/arm-frc-linux-gnueabi/arm-frc-linux-gnueabi.BUILD',
  sha256 = '9e2e58f0a668c22e46486a76df8b9da08f526cd8bf4e579f19b461f70a358bf8',
  url = 'http://frc971.org/Build-Dependencies/roborio-compiler-2017.tar.xz',
)

# Recompressed version of the one downloaded from Linaro at
# <https://releases.linaro.org/15.05/components/toolchain/binaries/arm-linux-gnueabihf/gcc-linaro-4.9-2015.05-x86_64_arm-linux-gnueabihf.tar.xz>,
# with workarounds for <https://github.com/bazelbuild/bazel/issues/574> and the
# top-level folder stripped off.
new_http_archive(
  name = 'linaro_linux_gcc_4_9_repo',
  build_file = 'compilers/linaro_linux_gcc_4.9.BUILD',
  sha256 = '25e97bcb0af4fd7cd626d5bb1b303c7d2cb13acf2474e335e3d431d1a53fbb52',
  url = 'http://frc971.org/Build-Dependencies/gcc-linaro-4.9-2015.05-x86_64_arm-linux-gnueabihf.tar.gz',
)

new_git_repository(
  name = 'python_gflags_repo',
  remote = 'https://github.com/gflags/python-gflags.git',
  build_file = 'debian/gflags.BUILD',
  commit = '41c4571864f0db5823e07715317e7388e94faabc',
)

bind(
  name = 'python-gflags',
  actual = '@python_gflags_repo//:gflags',
)

new_http_archive(
  name = 'python_glog_repo',
  build_file = 'debian/glog.BUILD',
  sha256 = '953fd80122c48023d1148e6d1bda2763fcab59c8a81682bb298238a5935547b0',
  url = 'https://pypi.python.org/packages/source/g/glog/glog-0.1.tar.gz',
  strip_prefix = 'glog-0.1',
)

bind(
  name = 'python-glog',
  actual = '@python_glog_repo//:glog',
)

new_http_archive(
  name = 'allwpilib_ni_libraries_repo_2016',
  build_file = 'debian/ni-libraries-2016.BUILD',
  sha256 = '821687afbee2d7531fb3e47d8d58ac10005695e59685be3ac3aa00b3179faf52',
  url = 'http://frc971.org/Build-Dependencies/allwpilib_ni-libraries_20749ed.tar.gz',
  strip_prefix = 'ni-libraries',
)

bind(
  name = 'ni-libraries-2016',
  actual = '@allwpilib_ni_libraries_repo_2016//:ni-libraries',
)

new_http_archive(
  name = 'allwpilib_ni_libraries_repo_2017',
  build_file = 'debian/ni-libraries-2017.BUILD',
  sha256 = '67c1ad365fb712cc0acb0bf43465b831030523dc6f88daa02626994f644d91eb',
  url = 'http://frc971.org/Build-Dependencies/allwpilib_ni-libraries_e375b4a.tar.gz',
  strip_prefix = 'ni-libraries',
)

bind(
  name = 'ni-libraries-2017',
  actual = '@allwpilib_ni_libraries_repo_2017//:ni-libraries',
)

# Downloaded from:
# https://pypi.python.org/packages/source/s/six/six-1.10.0.tar.gz
new_http_archive(
  name = 'six_repo',
  build_file = 'debian/six.BUILD',
  sha256 = '105f8d68616f8248e24bf0e9372ef04d3cc10104f1980f54d57b2ce73a5ad56a',
  url = 'http://frc971.org/Build-Dependencies/six-1.10.0.tar.gz',
  strip_prefix = 'six-1.10.0',
)

# For protobuf. Don't use these.
bind(
  name = 'six',
  actual = '@six_repo//:six',
)
bind(
  name = 'gtest',
  actual = '//third_party/googletest:googlemock',
)
bind(
  name = 'gtest_main',
  actual = '//third_party/googletest:googlemock_main',
)

new_http_archive(
  name = 'python_import_helpers',
  build_file = 'third_party/python_import_helpers.BUILD',
  url = 'http://frc971.org/Build-Dependencies/empty.tar.gz',
  sha256 = '71939a7d75585a57d2e99a33d39f391764d8f930f9a16acf32e00c5d3f432aa0',
)
