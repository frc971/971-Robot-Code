workspace(name = "org_frc971")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load(
    "//debian:python.bzl",
    python_debs = "files",
)
load(
    "//debian:clang.bzl",
    clang_debs = "files",
)
load(
    "//debian:patch.bzl",
    patch_debs = "files",
)
load(
    "//debian:pandoc.bzl",
    pandoc_debs = "files",
)
load(
    "//debian:libusb.bzl",
    libusb_debs = "files",
)
load(
    "//debian:mingw_compiler.bzl",
    mingw_compiler_debs = "files",
)
load(
    "//debian:patchelf.bzl",
    patchelf_debs = "files",
)
load(
    "//debian:matplotlib.bzl",
    matplotlib_debs = "files",
)
load(
    "//debian:arm_frc_gnueabi_deps.bzl",
    arm_frc_gnueabi_deps_debs = "files",
)
load(
    "//debian:python_gtk.bzl",
    python_gtk_debs = "files",
)
load(
    "//debian:ruby.bzl",
    ruby_debs = "files",
)
load("//debian:packages.bzl", "generate_repositories_for_debs")

generate_repositories_for_debs(python_debs)

generate_repositories_for_debs(clang_debs)

generate_repositories_for_debs(patch_debs)

generate_repositories_for_debs(pandoc_debs)

generate_repositories_for_debs(libusb_debs)

generate_repositories_for_debs(mingw_compiler_debs)

generate_repositories_for_debs(patchelf_debs)

generate_repositories_for_debs(matplotlib_debs)

generate_repositories_for_debs(arm_frc_gnueabi_deps_debs)

generate_repositories_for_debs(python_gtk_debs)

generate_repositories_for_debs(ruby_debs)

new_http_archive(
    name = "python_repo",
    build_file = "debian/python.BUILD",
    sha256 = "4ff939f90cffd8c72f9992d7420481e361b6016b0ce5c6fa701be0691d4e20fa",
    url = "http://www.frc971.org/Build-Dependencies/python-2.tar.gz",
)

new_http_archive(
    name = "clang_3p6_repo",
    build_file = "tools/cpp/clang_3p6/clang_3p6.BUILD",
    sha256 = "5ee9e04c55c2c99d0c0f83722102a49e98f485fc274f73111b33a7ac4e34e03e",
    url = "http://www.frc971.org/Build-Dependencies/clang_3p6.tar.gz",
)

new_local_repository(
    name = "usr_repo",
    build_file = "debian/usr.BUILD",
    path = "/usr",
)

new_git_repository(
    name = "slycot_repo",
    build_file = "@//debian:slycot.BUILD",
    commit = "5af5f283cb23cbe23c4dfea4d5e56071bdbd6e70",
    remote = "https://github.com/avventi/Slycot.git",
)

new_http_archive(
    name = "ruby_repo",
    build_file = "debian/ruby.BUILD",
    sha256 = "d3e21cca0abcad933de0d4095da35344a60475d1f5828ee99283ed4250ee1320",
    url = "http://www.frc971.org/Build-Dependencies/ruby.tar.gz",
)

new_http_archive(
    name = "arm_frc_linux_gnueabi_repo",
    build_file = "tools/cpp/arm-frc-linux-gnueabi/arm-frc-linux-gnueabi.BUILD",
    sha256 = "875b23bec5138e09e3d21cc1ff2727ea3ecbec57509c37589514ba50f92979c7",
    url = "http://www.frc971.org/Build-Dependencies/roborio-compiler-2018.tar.xz",
)

# Recompressed version of the one downloaded from Linaro at
# <https://releases.linaro.org/15.05/components/toolchain/binaries/arm-linux-gnueabihf/gcc-linaro-4.9-2015.05-x86_64_arm-linux-gnueabihf.tar.xz>,
# with workarounds for <https://github.com/bazelbuild/bazel/issues/574> and the
# top-level folder stripped off.
new_http_archive(
    name = "linaro_linux_gcc_4_9_repo",
    build_file = "compilers/linaro_linux_gcc_4.9.BUILD",
    sha256 = "25e97bcb0af4fd7cd626d5bb1b303c7d2cb13acf2474e335e3d431d1a53fbb52",
    url = "http://www.frc971.org/Build-Dependencies/gcc-linaro-4.9-2015.05-x86_64_arm-linux-gnueabihf.tar.gz",
)

new_git_repository(
    name = "python_gflags_repo",
    build_file = "@//debian:gflags.BUILD",
    commit = "41c4571864f0db5823e07715317e7388e94faabc",
    remote = "https://github.com/gflags/python-gflags.git",
)

bind(
    name = "python-gflags",
    actual = "@python_gflags_repo//:gflags",
)

local_repository(
    name = "com_github_gflags_gflags",
    path = "third_party/gflags",
)

new_http_archive(
    name = "python_glog_repo",
    build_file = "debian/glog.BUILD",
    sha256 = "953fd80122c48023d1148e6d1bda2763fcab59c8a81682bb298238a5935547b0",
    strip_prefix = "glog-0.1",
    url = "https://pypi.python.org/packages/source/g/glog/glog-0.1.tar.gz",
)

bind(
    name = "python-glog",
    actual = "@python_glog_repo//:glog",
)

new_http_archive(
    name = "allwpilib_ni_libraries_repo_2018",
    build_file = "debian/ni-libraries-2018.BUILD",
    sha256 = "05ef6701c77b83163b443aa956d151028861cc3fa29fdf2b6b77431b4a91bfb9",
    strip_prefix = "ni-libraries",
    url = "http://www.frc971.org/Build-Dependencies/allwpilib_ni-libraries_57e9fb3.tar.gz",
)

# Generated with:
# git fetch https://github.com/wpilibsuite/ni-libraries master
# git archive --output=allwpilib_ni-libraries_4785480.tar.gz --format=tar.gz 4785480
new_http_archive(
    name = "allwpilib_ni_libraries_2019",
    build_file = "debian/ni-libraries-2019.BUILD",
    sha256 = "2cdcde3391f36877b7533e15d0f36baf696b27c1107b77192a8200e26f13278c",
    url = "http://www.frc971.org/Build-Dependencies/allwpilib_ni-libraries_4785480.tar.gz",
)

# Downloaded from:
# https://pypi.python.org/packages/source/s/six/six-1.10.0.tar.gz
new_http_archive(
    name = "six_repo",
    build_file = "debian/six.BUILD",
    sha256 = "105f8d68616f8248e24bf0e9372ef04d3cc10104f1980f54d57b2ce73a5ad56a",
    strip_prefix = "six-1.10.0",
    url = "http://www.frc971.org/Build-Dependencies/six-1.10.0.tar.gz",
)

# For protobuf. Don't use these.
bind(
    name = "six",
    actual = "@six_repo//:six",
)

bind(
    name = "gtest",
    actual = "//third_party/googletest:googlemock",
)

bind(
    name = "gtest_main",
    actual = "//third_party/googletest:googlemock_main",
)

# Recompressed from libusb-1.0.21.7z.
http_file(
    name = "libusb_1_0_windows",
    sha256 = "fc2ba03992f343aabbaf9eb90559c6e00cdc6a2bd914d7cebea85857d5244015",
    url = "http://www.frc971.org/Build-Dependencies/libusb-1.0.21-windows.tar.xz",
)

# The data tarball of the same-named Debian package.
new_http_archive(
    name = "f2c",
    build_file = "debian/f2c.BUILD",
    sha256 = "2c677437f8217a2e2b23e41b33995d0571644fc1bea46de858f8913a5053e3f4",
    url = "http://www.frc971.org/Build-Dependencies/f2c_20100827-1_amd64.xz.tar.xz",
)

# Downloaded from http://www.netlib.org/clapack/.
new_http_archive(
    name = "clapack",
    build_file = "debian/clapack.BUILD",
    sha256 = "6dc4c382164beec8aaed8fd2acc36ad24232c406eda6db462bd4c41d5e455fac",
    strip_prefix = "CLAPACK-3.2.1/",
    url = "http://www.frc971.org/Build-Dependencies/clapack-3.2.1.tgz",
)

new_http_archive(
    name = "patch",
    build_file = "debian/patch.BUILD",
    sha256 = "b5ce139648a2e04f5585948ddad2fdae24dd4ee7976ac5a22d6ae7bd5674631e",
    url = "http://www.frc971.org/Build-Dependencies/patch.tar.gz",
)

new_http_archive(
    name = "pandoc",
    build_file = "debian/pandoc.BUILD",
    sha256 = "9f7a7adb3974a1f14715054c349ff3edc2909e920dbe3438fca437a83845f3c4",
    url = "http://www.frc971.org/Build-Dependencies/pandoc.tar.gz",
)

new_http_archive(
    name = "libusb",
    build_file = "debian/libusb.BUILD",
    sha256 = "3ca5cc2d317226f6646866ff9e8c443db3b0f6c82f828e800240982727531590",
    url = "http://www.frc971.org/Build-Dependencies/libusb.tar.gz",
)

new_http_archive(
    name = "mingw_compiler",
    build_file = "debian/mingw_compiler.BUILD",
    sha256 = "45e86a8460f2151a4f0306e7ae7b06761029d2412ee16f63d1e8d2d29354e378",
    url = "http://www.frc971.org/Build-Dependencies/mingw_compiler.tar.gz",
)

new_http_archive(
    name = "matplotlib",
    build_file = "debian/matplotlib.BUILD",
    sha256 = "dc8e04123a93180bf89727bf6b5a5a0f6d210b6c1c5eaec148f7f8183abbce24",
    url = "http://www.frc971.org/Build-Dependencies/matplotlib-2.tar.gz",
)

new_http_archive(
    name = "patchelf",
    build_file = "debian/patchelf.BUILD",
    sha256 = "bf8b709909d7d9e30815dd228eeded7dc282e3ce3919d0589ccbb56ac8632abc",
    url = "http://www.frc971.org/Build-Dependencies/patchelf.tar.gz",
)

new_http_archive(
    name = "arm_frc_gnueabi_deps",
    build_file = "debian/arm_frc_gnueabi_deps.BUILD",
    sha256 = "4b26fe45010817dc136488ee1604ade21bd7c264c29f17d864fc6eba9d7442c4",
    url = "http://www.frc971.org/Build-Dependencies/arm_frc_gnueabi_deps.tar.gz",
)

new_http_archive(
    name = "python_gtk",
    build_file = "debian/python_gtk.BUILD",
    sha256 = "850f5c1521b94c5c049c44d9107cd8ae9110696fbf054d2cb48bae9620fd4d23",
    url = "http://www.frc971.org/Build-Dependencies/python_gtk.tar.gz",
)

# Downloaded from
# https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2018q2/gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2?revision=bc2c96c0-14b5-4bb4-9f18-bceb4050fee7?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,7-2018-q2-update
new_http_archive(
    name = "gcc_arm_none_eabi",
    build_file = "compilers/gcc_arm_none_eabi.BUILD",
    sha256 = "bb17109f0ee697254a5d4ae6e5e01440e3ea8f0277f2e8169bf95d07c7d5fe69",
    strip_prefix = "gcc-arm-none-eabi-7-2018-q2-update/",
    url = "http://www.frc971.org/Build-Dependencies/gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2",
)

new_http_archive(
    name = "cgal_repo",
    build_file = "debian/cgal.BUILD",
    sha256 = "d564dda558570344b4caa66c5bae2cdae9ef68e07829d64f5651b25f2c6a0e9e",
    url = "http://www.frc971.org/Build-Dependencies/cgal-dev-4.5-2.tar.gz",
)

# Java9 JDK.
new_http_archive(
    name = "openjdk_linux_archive",
    build_file_content = """
java_runtime(
    name = 'jdk',
    srcs = glob(['**']),
    visibility = ['//visibility:public']
)
""",
    sha256 = "f27cb933de4f9e7fe9a703486cf44c84bc8e9f138be0c270c9e5716a32367e87",
    strip_prefix = "zulu9.0.7.1-jdk9.0.7-linux_x64-allmodules",
    urls = [
        "http://www.frc971.org/Build-Dependencies/zulu9.0.7.1-jdk9.0.7-linux_x64-allmodules.tar.gz",
    ],
)

local_repository(
    name = "com_google_protobuf",
    path = "third_party/protobuf",
)

local_repository(
    name = "com_github_google_glog",
    path = "third_party/google-glog",
)
