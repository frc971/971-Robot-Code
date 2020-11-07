workspace(name = "org_frc971")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
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
    "//debian:rsync.bzl",
    rsync_debs = "files",
)
load(
    "//debian:ssh.bzl",
    ssh_debs = "files",
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
    "//debian:opencv_armhf.bzl",
    opencv_armhf_debs = "files",
)
load(
    "//debian:opencv_amd64.bzl",
    opencv_amd64_debs = "files",
)
load(
    "//debian:gstreamer_amd64.bzl",
    gstreamer_amd64_debs = "files",
)
load(
    "//debian:gstreamer_armhf.bzl",
    gstreamer_armhf_debs = "files",
)
load(
    "//debian:m4.bzl",
    m4_debs = "files",
)
load(
    "//debian:lzma_amd64.bzl",
    lzma_amd64_debs = "files",
)
load(
    "//debian:lzma_arm64.bzl",
    lzma_arm64_debs = "files",
)
load("//debian:packages.bzl", "generate_repositories_for_debs")

generate_repositories_for_debs(python_debs)

generate_repositories_for_debs(clang_debs)

generate_repositories_for_debs(rsync_debs)

generate_repositories_for_debs(ssh_debs)

generate_repositories_for_debs(patch_debs)

generate_repositories_for_debs(pandoc_debs)

generate_repositories_for_debs(libusb_debs)

generate_repositories_for_debs(mingw_compiler_debs)

generate_repositories_for_debs(patchelf_debs)

generate_repositories_for_debs(matplotlib_debs)

generate_repositories_for_debs(arm_frc_gnueabi_deps_debs)

generate_repositories_for_debs(python_gtk_debs)

generate_repositories_for_debs(opencv_armhf_debs)

generate_repositories_for_debs(opencv_amd64_debs)

generate_repositories_for_debs(gstreamer_amd64_debs)

generate_repositories_for_debs(gstreamer_armhf_debs)

generate_repositories_for_debs(m4_debs)

generate_repositories_for_debs(lzma_amd64_debs)

generate_repositories_for_debs(lzma_arm64_debs)

http_archive(
    name = "python_repo",
    build_file = "@//debian:python.BUILD",
    sha256 = "cc48d18417f015238b7efb083f4ff64a80f0baf347fbaf94d5437c14f7a3d84c",
    url = "https://www.frc971.org/Build-Dependencies/python-4.tar.gz",
)

http_archive(
    name = "clang_6p0_repo",
    build_file = "@//tools/cpp/clang_6p0:clang_6p0.BUILD",
    sha256 = "7c5dc0f124fbd26e440797a851466e7f852da27d9f1562c74059b5a34c294cc9",
    url = "https://www.frc971.org/Build-Dependencies/clang_6p0.tar.gz",
)

local_repository(
    name = "com_google_absl",
    path = "third_party/abseil",
)

local_repository(
    name = "org_tuxfamily_eigen",
    path = "third_party/eigen",
)

# C++ rules for Bazel.
http_archive(
    name = "rules_cc",
    sha256 = "67412176974bfce3f4cf8bdaff39784a72ed709fc58def599d1f68710b58d68b",
    strip_prefix = "rules_cc-b7fe9697c0c76ab2fd431a891dbb9a6a32ed7c3e",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_cc/archive/b7fe9697c0c76ab2fd431a891dbb9a6a32ed7c3e.zip",
        "https://github.com/bazelbuild/rules_cc/archive/b7fe9697c0c76ab2fd431a891dbb9a6a32ed7c3e.zip",
    ],
)

new_local_repository(
    name = "usr_repo",
    build_file = "@//debian:usr.BUILD",
    path = "/usr",
)

new_git_repository(
    name = "slycot_repo",
    build_file = "@//debian:slycot.BUILD",
    commit = "5af5f283cb23cbe23c4dfea4d5e56071bdbd6e70",
    remote = "https://github.com/avventi/Slycot.git",
)

http_archive(
    name = "arm_frc_linux_gnueabi_repo",
    build_file = "@//tools/cpp/arm-frc-linux-gnueabi:arm-frc-linux-gnueabi.BUILD",
    sha256 = "043a5b047c2af9cf80d146d8327b588264c98a01e0f3f41e3564dd2bbbc95c0e",
    strip_prefix = "frc2020/roborio/",
    url = "https://www.frc971.org/Build-Dependencies/FRC-2020-Linux-Toolchain-7.3.0.tar.gz",
)

# Recompressed version of the one downloaded from Linaro at
# <https://releases.linaro.org/components/toolchain/binaries/7.4-2019.02/arm-linux-gnueabihf/gcc-linaro-7.4.1-2019.02-x86_64_arm-linux-gnueabihf.tar.xz>
# with workarounds for <https://github.com/bazelbuild/bazel/issues/574> and the
# top-level folder stripped off.
http_archive(
    name = "linaro_linux_gcc_repo",
    build_file = "@//:compilers/linaro_linux_gcc.BUILD",
    sha256 = "3c951cf1941d0fa06d64cc0d5e88612b209d8123b273fa26c16d70bd7bc6b163",
    strip_prefix = "gcc-linaro-7.4.1-2019.02-x86_64_arm-linux-gnueabihf",
    url = "https://www.frc971.org/Build-Dependencies/gcc-linaro-7.4.1-2019.02-x86_64_arm-linux-gnueabihf.tar.xz",
)

# The main partition from https://downloads.raspberrypi.org/raspbian_lite/images/raspbian_lite-2019-09-30/2019-09-26-raspbian-buster-lite.zip.
# The following folders are removed to make bazel happy with it:
#   usr/share/ca-certificates
# This copy command to make clang happy: `cp usr/lib/arm-linux-gnueabihf/*.o usr/lib`
http_archive(
    name = "armhf_debian_rootfs",
    build_file = "@//:compilers/debian_rootfs.BUILD",
    sha256 = "8c827bdb79615046ee3e13e85664e5d01286ca1721f7169341667a634e599eb6",
    url = "https://www.frc971.org/Build-Dependencies/2019-09-26-raspbian-buster-lite_rootfs.tar.bz2",
)

# Created with:
#   `debootstrap buster buster_sysroot`
# and then chrooting in and running:
#   apt install libc6-dev libstdc++-7-dev
# and then tarring up the result
http_archive(
    name = "amd64_debian_sysroot",
    build_file = "@//:compilers/debian_rootfs.BUILD",
    sha256 = "fb0a4f0b35b4c99fcfc83902d2d2eaac7062024b2ff2f998e68736aac92c8e59",
    url = "https://www.frc971.org/Build-Dependencies/2019-01-14-debian-buster_rootfs.tar.bz2",
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

# Downloaded from:
# https://pypi.python.org/packages/source/g/glog/glog-0.1.tar.gz
http_archive(
    name = "python_glog_repo",
    build_file = "@//debian:glog.BUILD",
    sha256 = "953fd80122c48023d1148e6d1bda2763fcab59c8a81682bb298238a5935547b0",
    strip_prefix = "glog-0.1",
    url = "https://www.frc971.org/Build-Dependencies/glog-0.1.tar.gz",
)

bind(
    name = "python-glog",
    actual = "@python_glog_repo//:glog",
)

# Generated with:
# git fetch https://github.com/wpilibsuite/ni-libraries master
# git archive --output=allwpilib_ni-libraries_c826046.tar.gz --format=tar.gz c826046
http_archive(
    name = "allwpilib_ni_libraries",
    build_file = "@//debian:ni-libraries.BUILD",
    sha256 = "59e971854d689b8e60d2b7ede3cc1da911dbc70deeccb9b5306bb7c7aa5102d9",
    url = "https://www.frc971.org/Build-Dependencies/allwpilib_ni-libraries_c826046.tar.gz",
)

# Downloaded from:
# https://pypi.python.org/packages/source/s/six/six-1.10.0.tar.gz
http_archive(
    name = "six_repo",
    build_file = "@//debian:six.BUILD",
    sha256 = "105f8d68616f8248e24bf0e9372ef04d3cc10104f1980f54d57b2ce73a5ad56a",
    strip_prefix = "six-1.10.0",
    url = "https://www.frc971.org/Build-Dependencies/six-1.10.0.tar.gz",
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
    downloaded_file_path = "libusb-1.0.21-windows.tar.xz",
    sha256 = "fc2ba03992f343aabbaf9eb90559c6e00cdc6a2bd914d7cebea85857d5244015",
    urls = ["https://www.frc971.org/Build-Dependencies/libusb-1.0.21-windows.tar.xz"],
)

# The data tarball of the same-named Debian package.
http_archive(
    name = "f2c",
    build_file = "@//debian:f2c.BUILD",
    sha256 = "2c677437f8217a2e2b23e41b33995d0571644fc1bea46de858f8913a5053e3f4",
    url = "https://www.frc971.org/Build-Dependencies/f2c_20100827-1_amd64.xz.tar.xz",
)

# Downloaded from http://www.netlib.org/clapack/.
http_archive(
    name = "clapack",
    build_file = "@//debian:clapack.BUILD",
    sha256 = "6dc4c382164beec8aaed8fd2acc36ad24232c406eda6db462bd4c41d5e455fac",
    strip_prefix = "CLAPACK-3.2.1/",
    url = "https://www.frc971.org/Build-Dependencies/clapack-3.2.1.tgz",
)

http_archive(
    name = "patch",
    build_file = "@//debian:patch.BUILD",
    sha256 = "b5ce139648a2e04f5585948ddad2fdae24dd4ee7976ac5a22d6ae7bd5674631e",
    url = "https://www.frc971.org/Build-Dependencies/patch.tar.gz",
)

http_archive(
    name = "rsync",
    build_file = "@//debian:rsync.BUILD",
    sha256 = "53be65a9214aaa6d1b9176f135184fb4a78ccefd58f95ce0da37e6a392dfeb60",
    url = "https://www.frc971.org/Build-Dependencies/rsync.tar.gz",
)

# //debian:ssh
http_archive(
    name = "ssh",
    build_file = "@//debian:ssh.BUILD",
    sha256 = "470fdc1252a2133a9d3c3da778e892a5b88f04f402cb04d8eb1cff7853242034",
    url = "https://www.frc971.org/Build-Dependencies/ssh_v3.tar.gz",
)

http_archive(
    name = "pandoc",
    build_file = "@//debian:pandoc.BUILD",
    sha256 = "9f7a7adb3974a1f14715054c349ff3edc2909e920dbe3438fca437a83845f3c4",
    url = "https://www.frc971.org/Build-Dependencies/pandoc.tar.gz",
)

http_archive(
    name = "libusb",
    build_file = "@//debian:libusb.BUILD",
    sha256 = "3ca5cc2d317226f6646866ff9e8c443db3b0f6c82f828e800240982727531590",
    url = "https://www.frc971.org/Build-Dependencies/libusb.tar.gz",
)

http_archive(
    name = "mingw_compiler",
    build_file = "@//debian:mingw_compiler.BUILD",
    sha256 = "45e86a8460f2151a4f0306e7ae7b06761029d2412ee16f63d1e8d2d29354e378",
    url = "https://www.frc971.org/Build-Dependencies/mingw_compiler.tar.gz",
)

# Note that we should generally keep the matplotlib repo in a folder not
# named matplotlib, because otherwise the repository itself tends to end up
# on the PYTHONPATH, rather than the matplotlib folder within this repo.
http_archive(
    name = "matplotlib_repo",
    build_file = "@//debian:matplotlib.BUILD",
    sha256 = "24f8b75754e465299ddf92bd895ab111d54945a45b0f410d7cfa16b15b162e2f",
    url = "https://www.frc971.org/Build-Dependencies/matplotlib-4.tar.gz",
)

http_archive(
    name = "patchelf",
    build_file = "@//debian:patchelf.BUILD",
    sha256 = "bf8b709909d7d9e30815dd228eeded7dc282e3ce3919d0589ccbb56ac8632abc",
    url = "https://www.frc971.org/Build-Dependencies/patchelf.tar.gz",
)

http_archive(
    name = "arm_frc_gnueabi_deps",
    build_file = "@//debian:arm_frc_gnueabi_deps.BUILD",
    sha256 = "4b26fe45010817dc136488ee1604ade21bd7c264c29f17d864fc6eba9d7442c4",
    url = "https://www.frc971.org/Build-Dependencies/arm_frc_gnueabi_deps.tar.gz",
)

http_archive(
    name = "python_gtk",
    build_file = "@//debian:python_gtk.BUILD",
    sha256 = "2c95dbce3ead87ad7d64327535f08c95ab14d5c12d5646530d734e70a1921130",
    url = "https://www.frc971.org/Build-Dependencies/python_gtk-2.tar.gz",
)

# Downloaded from
# https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2018q2/gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2?revision=bc2c96c0-14b5-4bb4-9f18-bceb4050fee7?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,7-2018-q2-update
http_archive(
    name = "gcc_arm_none_eabi",
    build_file = "@//:compilers/gcc_arm_none_eabi.BUILD",
    sha256 = "bb17109f0ee697254a5d4ae6e5e01440e3ea8f0277f2e8169bf95d07c7d5fe69",
    strip_prefix = "gcc-arm-none-eabi-7-2018-q2-update/",
    url = "https://www.frc971.org/Build-Dependencies/gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2",
)

http_archive(
    name = "cgal_repo",
    build_file = "@//debian:cgal.BUILD",
    sha256 = "d564dda558570344b4caa66c5bae2cdae9ef68e07829d64f5651b25f2c6a0e9e",
    url = "https://www.frc971.org/Build-Dependencies/cgal-dev-4.5-2.tar.gz",
)

# Java9 JDK.
http_archive(
    name = "openjdk_linux_archive",
    build_file_content = """
java_runtime(
    name = 'jdk',
    srcs = glob(['**']),
    visibility = ['//visibility:public'],
)
""",
    sha256 = "f27cb933de4f9e7fe9a703486cf44c84bc8e9f138be0c270c9e5716a32367e87",
    strip_prefix = "zulu9.0.7.1-jdk9.0.7-linux_x64-allmodules",
    urls = [
        "https://www.frc971.org/Build-Dependencies/zulu9.0.7.1-jdk9.0.7-linux_x64-allmodules.tar.gz",
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

local_repository(
    name = "com_google_googletest",
    path = "third_party/googletest",
)

# External dependency: Google Benchmark; has no Bazel build.
local_repository(
    name = "com_github_google_benchmark",
    path = "third_party/google-benchmark",
)

local_repository(
    name = "com_google_ceres_solver",
    path = "third_party/ceres",
)

# Downloaded from http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/api-cpp/5.18.1/.
http_archive(
    name = "ctre_phoenix_api_cpp_headers",
    build_file_content = """
cc_library(
    name = 'api-cpp',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/phoenix/**/*.h']),
)
""",
    sha256 = "230ff927e36b2f75e746da0f0bf9852e5a049bb3e95c4617138ef0618b2e80d9",
    urls = [
        "https://www.frc971.org/Build-Dependencies/api-cpp-5.18.1-headers.zip",
    ],
)

# Downloaded from http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/api-cpp/5.18.1/.
http_archive(
    name = "ctre_phoenix_api_cpp_athena",
    build_file_content = """
cc_library(
    name = 'api-cpp',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/static/libCTRE_Phoenix.a'],
    restricted_to = ['@//tools:roborio'],
    deps = [
      '@ctre_phoenix_core_headers//:core',
      '@ctre_phoenix_core_athena//:core',
    ],
)
""",
    sha256 = "a4de1930e1e946e1c72c13ee272dae38c12c4b7b85b44477dbb67312724d96b1",
    urls = [
        "https://www.frc971.org/Build-Dependencies/api-cpp-5.18.1-linuxathenastatic.zip",
    ],
)

# Downloaded from http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/diagnostics/5.18.1/.
http_archive(
    name = "ctre_phoenix_diagnostics_headers",
    build_file_content = """
cc_library(
    name = 'diagnostics',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/phoenix/**/*.h']),
)
""",
    sha256 = "a94bff6c241de8dc2396a1cece9e2822fe4a7e4980aedaaea682c3e8c5de008c",
    urls = [
        "https://www.frc971.org/Build-Dependencies/diagnostics-5.18.1-headers.zip",
    ],
)

# Downloaded from http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/diagnostics/5.18.1/.
http_archive(
    name = "ctre_phoenix_diagnostics_athena",
    build_file_content = """
cc_library(
    name = 'diagnostics',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/static/libCTRE_PhoenixDiagnostics.a'],
    restricted_to = ['@//tools:roborio'],
    deps = [
      '@ctre_phoenix_core_headers//:core',
      '@ctre_phoenix_core_athena//:core',
    ],
)
""",
    sha256 = "638a4a4d7400942baa040619ea6cde2bdef0e7721300a9427424a577ce0f56db",
    urls = [
        "https://www.frc971.org/Build-Dependencies/diagnostics-5.18.1-linuxathenastatic.zip",
    ],
)

# Downloaded from http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/cci/5.18.1/.
http_archive(
    name = "ctre_phoenix_cci_headers",
    build_file_content = """
cc_library(
    name = 'cci',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/phoenix/**/*.h']),
)
""",
    sha256 = "31e4d8f7fd9612ba687661e19aabc3d89dc076f66756d4696aa7799f31bbc72f",
    urls = [
        "https://www.frc971.org/Build-Dependencies/cci-5.18.1-headers.zip",
    ],
)

# Downloaded from http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/cci/5.18.1/.
http_archive(
    name = "ctre_phoenix_cci_athena",
    build_file_content = """
cc_library(
    name = 'cci',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/static/libCTRE_PhoenixCCI.a'],
    restricted_to = ['@//tools:roborio'],
)
""",
    sha256 = "e5d9b58072002dbd2daa8cc8d42e047e5c90d26bd5a2b1d63dc1b89112ac3837",
    urls = [
        "https://www.frc971.org/Build-Dependencies/cci-5.18.1-linuxathenastatic.zip",
    ],
)

# Downloaded from http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/core/5.18.1/.
http_archive(
    name = "ctre_phoenix_core_headers",
    build_file_content = """
cc_library(
    name = 'core',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/phoenix/**/*.h']),
)
""",
    sha256 = "af2db0f9c3693cbb74216882ee140e4d6b722a416f2d384062378a8ae37f65ee",
    urls = [
        "https://www.frc971.org/Build-Dependencies/core-5.18.1-headers.zip",
    ],
)

# Downloaded from http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/core/5.18.1/.
http_archive(
    name = "ctre_phoenix_core_athena",
    build_file_content = """
cc_library(
    name = 'core',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/static/libCTRE_PhoenixCore.a'],
    restricted_to = ['@//tools:roborio'],
)
""",
    sha256 = "cd827bc68c0f4ef2fe6c363a7f9f5a08f7d944b574c65a2c7fb823686501f43f",
    urls = [
        "https://www.frc971.org/Build-Dependencies/core-5.18.1-linuxathenastatic.zip",
    ],
)

http_archive(
    name = "build_bazel_rules_typescript",
    strip_prefix = "rules_typescript-0.21.0",
    url = "https://github.com/bazelbuild/rules_typescript/archive/0.21.0.zip",
)

emscripten_version = "1.38.31"

http_archive(
    name = "emscripten_toolchain",
    build_file_content = """
filegroup(
    name = 'all',
    visibility = ['//visibility:public'],
    srcs = glob(['**']),
)
""",
    # TODO(james): Once a functioning release contains this patch, convert
    # to that. See https://github.com/emscripten-core/emscripten/pull/9048
    patches = ["@//debian:emscripten_toolchain.patch"],
    sha256 = "c87e42cb6a104094e7daf2b7e61ac835f83674ac0168f533455838a1129cc764",
    strip_prefix = "emscripten-" + emscripten_version,
    urls = ["https://github.com/emscripten-core/emscripten/archive/" + emscripten_version + ".tar.gz"],
)

new_http_archive(
    name = "emscripten_clang",
    build_file_content = """
filegroup(
    name = 'all',
    visibility = ['//visibility:public'],
    srcs = glob(['**']),
)
""",
    sha256 = "a0c2f2c5a897577f40af0fdf68dcf3cf65557ff20c081df26678c066a4fed4b1",
    strip_prefix = "emscripten-llvm-e" + emscripten_version,
    url = "https://www.frc971.org/Build-Dependencies/emscripten-llvm-e" + emscripten_version + ".tar.gz",
)

new_http_archive(
    name = "webrtc_x64",
    build_file = "@//debian:webrtc.BUILD",
    sha256 = "bd212b2a112a043d08d27f49027091788fa01c7c2ac5f072d096c17d9dbd976f",
    url = "https://www.frc971.org/Build-Dependencies/webrtc-30326-1a68679-linux-x64.tar.gz",
)

new_http_archive(
    name = "webrtc_arm",
    build_file = "@//debian:webrtc.BUILD",
    sha256 = "c34badaf313877cd03a0dfd6b71de024d806a7652550a7f1cd7dea523a7c813d",
    url = "https://www.frc971.org/Build-Dependencies/webrtc-30326-1a68679-linux-arm.tar.gz",
)

new_http_archive(
    name = "webrtc_rio",
    build_file = "@//debian:webrtc.BUILD",
    sha256 = "d86d3b030099b35ae5ea31c807fb4d0b0352598e79f1ea84877e5504e185faa8",
    url = "https://www.frc971.org/Build-Dependencies/webrtc-30376-4c4735b-linux-rio.tar.gz",
)

# Fetch our Bazel dependencies that aren't distributed on npm
load("@build_bazel_rules_typescript//:package.bzl", "rules_typescript_dependencies")

rules_typescript_dependencies()

# Setup TypeScript toolchain
load("@build_bazel_rules_typescript//:defs.bzl", "ts_setup_workspace")

ts_setup_workspace()

# Setup the NodeJS toolchain
load("@build_bazel_rules_nodejs//:defs.bzl", "node_repositories", "yarn_install")

node_repositories()

# Setup Bazel managed npm dependencies with the `yarn_install` rule.
yarn_install(
    name = "npm",
    package_json = "//:package.json",
    yarn_lock = "//:yarn.lock",
)

# Flatbuffers
local_repository(
    name = "com_github_google_flatbuffers",
    path = "third_party/flatbuffers",
)

http_file(
    name = "sample_logfile",
    downloaded_file_path = "log.fbs",
    sha256 = "91c98edee0c90a19992792c711dde4a6743af2d6d7e45b5079ec228fdf51ff11",
    urls = ["https://www.frc971.org/Build-Dependencies/small_sample_logfile.fbs"],
)

http_file(
    name = "drivetrain_replay",
    downloaded_file_path = "spinning_wheels_while_still.bfbs",
    sha256 = "d724dbf0acae894b30c9bb62006a2633a7f4c478db48548e76cbec03cbb07f46",
    urls = ["https://www.frc971.org/Build-Dependencies/spinning_wheels_while_still3.bfbs"],
)

# OpenCV armhf (for raspberry pi)
http_archive(
    name = "opencv_armhf",
    build_file = "@//debian:opencv.BUILD",
    sha256 = "1dd496ad0947ed6ce5d89cbefcfa55ea15ccb5bf70fa6ad7701c62cf2fcdd657",
    url = "https://www.frc971.org/Build-Dependencies/opencv_armhf_v3.tar.gz",
)

http_archive(
    name = "opencv_k8",
    build_file = "@//debian:opencv.BUILD",
    sha256 = "5ae1b473c2d47576d6cbea61d46423d8782936c057b578f1236809e43d0a62d1",
    url = "https://www.frc971.org/Build-Dependencies/opencv_amd64_v2.tar.gz",
)

# Downloaded from:
# https://github.com/halide/Halide/releases/download/release_2019_08_27/halide-linux-64-gcc53-800-65c26cba6a3eca2d08a0bccf113ca28746012cc3.tgz
# which is "Halide 2019/08/27" at https://github.com/halide/Halide/releases.
http_archive(
    name = "halide_k8",
    build_file = "@//debian:halide.BUILD",
    sha256 = "c67185d50a99adba86f6b2cc43c7e2cf11bcdfba9052d05e764a89b456a50446",
    strip_prefix = "halide/",
    url = "https://www.frc971.org/Build-Dependencies/halide-linux-64-gcc53-800-65c26cba6a3eca2d08a0bccf113ca28746012cc3.tgz",
)

# Downloaded from:
# https://github.com/halide/Halide/releases/download/release_2019_08_27/halide-arm32-linux-32-trunk-65c26cba6a3eca2d08a0bccf113ca28746012cc3.tgz
# which is "Halide 2019/08/27" at https://github.com/halide/Halide/releases.
http_archive(
    name = "halide_armhf",
    build_file = "@//debian:halide.BUILD",
    sha256 = "10564c559c9e04a173823413916d05fadd6e697d91bab21ddc5041190fa8f0f0",
    strip_prefix = "halide/",
    url = "https://www.frc971.org/Build-Dependencies/halide-arm32-linux-32-trunk-65c26cba6a3eca2d08a0bccf113ca28746012cc3.tgz",
)

# Downloaded from:
# https://files.pythonhosted.org/packages/05/23/7f9a896da9e7ce4170377a7a14bb804b460761f9dd66734e6ad8f001a76c/opencv_contrib_python_nonfree-4.1.1.1-cp35-cp35m-manylinux1_x86_64.whl
http_archive(
    name = "opencv_contrib_nonfree_amd64",
    build_file = "@//debian:opencv_python.BUILD",
    sha256 = "c10e7712ee1f19bf382c64fc29b5d24fa0b5bfd901eab69cef83604713e6a89e",
    type = "zip",
    url = "https://www.frc971.org/Build-Dependencies/opencv_contrib_python_nonfree-4.1.1.1-cp35-cp35m-manylinux1_x86_64.whl",
)

http_archive(
    name = "gstreamer_k8",
    build_file = "@//debian:gstreamer.BUILD",
    sha256 = "4d74d4a82f7a73dc9fe9463d5fae409b17845eef7cd64ef9c4c4553816c53589",
    url = "https://www.frc971.org/Build-Dependencies/gstreamer_amd64.tar.gz",
)

http_archive(
    name = "gstreamer_armhf",
    build_file = "@//debian:gstreamer.BUILD",
    sha256 = "c5ac4c604952c274a50636e244f0d091bd1de302032446f24f0e9e03ae9c76f7",
    url = "https://www.frc971.org/Build-Dependencies/gstreamer_armhf.tar.gz",
)

# Downloaded from:
# https://files.pythonhosted.org/packages/64/a7/45e11eebf2f15bf987c3bc11d37dcc838d9dc81250e67e4c5968f6008b6c/Jinja2-2.11.2.tar.gz
http_archive(
    name = "python_jinja2",
    build_file = "@//debian:python_jinja2.BUILD",
    sha256 = "89aab215427ef59c34ad58735269eb58b1a5808103067f7bb9d5836c651b3bb0",
    strip_prefix = "Jinja2-2.11.2",
    url = "https://www.frc971.org/Build-Dependencies/Jinja2-2.11.2.tar.gz",
)

# Downloaded from:
# https://files.pythonhosted.org/packages/b9/2e/64db92e53b86efccfaea71321f597fa2e1b2bd3853d8ce658568f7a13094/MarkupSafe-1.1.1.tar.gz
http_archive(
    name = "python_markupsafe",
    build_file = "@//debian:python_markupsafe.BUILD",
    sha256 = "29872e92839765e546828bb7754a68c418d927cd064fd4708fab9fe9c8bb116b",
    strip_prefix = "MarkupSafe-1.1.1",
    url = "https://www.frc971.org/Build-Dependencies/MarkupSafe-1.1.1.tar.gz",
)

http_archive(
    name = "m4_v1.4.18",
    build_file = "@//debian:m4.BUILD",
    sha256 = "ee8dfe664ac8c1d066bab64f71bd076a021875581b3cc47dac4a14a475f50b15",
    url = "https://www.frc971.org/Build-Dependencies/m4.tar.gz",
)

# //debian:lzma_amd64
http_archive(
    name = "lzma_amd64",
    build_file_content = """
cc_library(
    name = "lib",
    srcs = [
        "usr/lib/x86_64-linux-gnu/liblzma.a",
    ],
    hdrs = glob([
        "usr/include/lzma/*.h",
        "usr/include/*.h",
    ]),
    strip_include_prefix = "usr/include",
    visibility = ["//visibility:public"],
)
""",
    sha256 = "e0ccaa7f793e44638e9f89570e00f146073a98a5928e0b547146c8184488bb19",
    urls = ["https://www.frc971.org/Build-Dependencies/lzma_amd64.tar.gz"],
)

# //debian:lzma_arm64
http_archive(
    name = "lzma_arm64",
    build_file_content = """
cc_library(
    name = "lib",
    srcs = [
        "usr/lib/aarch64-linux-gnu/liblzma.a",
    ],
    hdrs = glob([
        "usr/include/lzma/*.h",
        "usr/include/*.h",
    ]),
    strip_include_prefix = "usr/include",
    visibility = ["//visibility:public"],
)
""",
    sha256 = "18db35669ee49a5f8324a344071dd4ab553e716f385fb75747b909bd1de959f5",
    urls = ["https://www.frc971.org/Build-Dependencies/lzma_arm64.tar.gz"],
)
