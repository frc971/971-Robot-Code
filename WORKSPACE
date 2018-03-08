workspace(name = 'org_frc971')

load("//debian:python.bzl", python_debs="files")
load("//debian:packages.bzl", "generate_repositories_for_debs")

generate_repositories_for_debs(python_debs)

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

new_http_archive(
  name = 'arm_frc_linux_gnueabi_repo',
  build_file = 'tools/cpp/arm-frc-linux-gnueabi/arm-frc-linux-gnueabi.BUILD',
  sha256 = '875b23bec5138e09e3d21cc1ff2727ea3ecbec57509c37589514ba50f92979c7',
  url = 'http://frc971.org/Build-Dependencies/roborio-compiler-2018.tar.xz',
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
  name = 'allwpilib_ni_libraries_repo_2018',
  build_file = 'debian/ni-libraries-2018.BUILD',
  sha256 = '05ef6701c77b83163b443aa956d151028861cc3fa29fdf2b6b77431b4a91bfb9',
  url = 'http://frc971.org/Build-Dependencies/allwpilib_ni-libraries_57e9fb3.tar.gz',
  strip_prefix = 'ni-libraries',
)

new_http_archive(
  name = 'allwpilib_ni_libraries_repo_2017',
  build_file = 'debian/ni-libraries-2017.BUILD',
  sha256 = '67c1ad365fb712cc0acb0bf43465b831030523dc6f88daa02626994f644d91eb',
  url = 'http://frc971.org/Build-Dependencies/allwpilib_ni-libraries_e375b4a.tar.gz',
  strip_prefix = 'ni-libraries',
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

new_local_repository(
  name = 'libusb',
  path = '/usr',
  build_file = 'debian/libusb.BUILD',
)

# Created by combining libusb-1.0-0_2%3a1.0.19-1_amd64,
# libusb-1.0-0-dev_2%3a1.0.19-1, and libudev1_215-17+deb8u7.
new_http_archive(
  name = 'libusb_1_0',
  build_file = 'debian/libusb-1.0.BUILD',
  url = 'http://frc971.org/Build-Dependencies/libusb-1.0-1.0.19.tar.xz',
  sha256 = '12acb30faacd10e9aa7f3a5e074701e167ce9bbd45694db37d13d55de5398816',
)

# Recompressed from libusb-1.0.21.7z.
http_file(
  name = 'libusb_1_0_windows',
  url = 'http://frc971.org/Build-Dependencies/libusb-1.0.21-windows.tar.xz',
  sha256 = 'fc2ba03992f343aabbaf9eb90559c6e00cdc6a2bd914d7cebea85857d5244015',
)

# The data tarball of the same-named Debian package.
new_http_archive(
    name = "f2c",
    sha256 = "2c677437f8217a2e2b23e41b33995d0571644fc1bea46de858f8913a5053e3f4",
    url = "http://frc971.org/Build-Dependencies/f2c_20100827-1_amd64.xz.tar.xz",
    build_file = "debian/f2c.BUILD",
)

# Downloaded from http://www.netlib.org/clapack/.
new_http_archive(
  name = 'clapack',
  url = "http://frc971.org/Build-Dependencies/clapack-3.2.1.tgz",
  build_file = 'debian/clapack.BUILD',
  sha256 = '6dc4c382164beec8aaed8fd2acc36ad24232c406eda6db462bd4c41d5e455fac',
  strip_prefix = 'CLAPACK-3.2.1/',
)
