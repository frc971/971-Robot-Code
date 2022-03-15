workspace(name = "org_frc971")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/jdk:remote_java_repository.bzl", "remote_java_repository")
load(
    "//debian:python.bzl",
    python_debs = "files",
)
load(
    "//debian:apache2.bzl",
    apache2_debs = "files",
)
load(
    "//debian:postgresql_amd64.bzl",
    postgresql_amd64_debs = "files",
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
    "//debian:opencv_arm64.bzl",
    opencv_arm64_debs = "files",
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
load(
    "//debian:libtinfo5_amd64.bzl",
    libtinfo5_amd64_debs = "files",
)
load("//debian:packages.bzl", "generate_repositories_for_debs")

generate_repositories_for_debs(python_debs)

generate_repositories_for_debs(rsync_debs)

generate_repositories_for_debs(ssh_debs)

generate_repositories_for_debs(apache2_debs)

generate_repositories_for_debs(postgresql_amd64_debs)

generate_repositories_for_debs(patch_debs)

generate_repositories_for_debs(pandoc_debs)

generate_repositories_for_debs(libusb_debs)

generate_repositories_for_debs(mingw_compiler_debs)

generate_repositories_for_debs(patchelf_debs)

generate_repositories_for_debs(matplotlib_debs)

generate_repositories_for_debs(arm_frc_gnueabi_deps_debs)

generate_repositories_for_debs(python_gtk_debs)

generate_repositories_for_debs(opencv_arm64_debs)

generate_repositories_for_debs(opencv_armhf_debs)

generate_repositories_for_debs(opencv_amd64_debs)

generate_repositories_for_debs(gstreamer_amd64_debs)

generate_repositories_for_debs(gstreamer_armhf_debs)

generate_repositories_for_debs(m4_debs)

generate_repositories_for_debs(lzma_amd64_debs)

generate_repositories_for_debs(lzma_arm64_debs)

generate_repositories_for_debs(libtinfo5_amd64_debs)

local_repository(
    name = "com_grail_bazel_toolchain",
    path = "third_party/bazel-toolchain",
)

load("@com_grail_bazel_toolchain//toolchain:rules.bzl", "llvm", "llvm_toolchain")

llvm_version = "13.0.0"

llvm(
    name = "llvm_k8",
    distribution = "clang+llvm-%s-x86_64-linux-gnu-ubuntu-16.04.tar.xz" % llvm_version,
    llvm_version = llvm_version,
)

llvm(
    name = "llvm_armv7",
    distribution = "clang+llvm-%s-armv7a-linux-gnueabihf.tar.xz" % llvm_version,
    llvm_version = llvm_version,
)

llvm(
    name = "llvm_aarch64",
    distribution = "clang+llvm-%s-aarch64-linux-gnu.tar.xz" % llvm_version,
    llvm_version = llvm_version,
)

llvm_conlyopts = [
    "-std=gnu99",
]

llvm_copts = [
    "-D__STDC_FORMAT_MACROS",
    "-D__STDC_CONSTANT_MACROS",
    "-D__STDC_LIMIT_MACROS",
    "-D_FILE_OFFSET_BITS=64",
    "-fmessage-length=100",
    "-fmacro-backtrace-limit=0",
    "-Wextra",
    "-Wpointer-arith",
    "-Wstrict-aliasing",
    "-Wcast-qual",
    "-Wcast-align",
    "-Wwrite-strings",
    "-Wtype-limits",
    "-Wsign-compare",
    "-Wformat=2",
    "-Werror",
    "-ggdb3",
]

llvm_cxxopts = [
    "-std=gnu++17",
]

llvm_opt_copts = [
    "-DAOS_DEBUG=0",
]

llvm_fastbuild_copts = [
    "-DAOS_DEBUG=0",
]

llvm_dbg_copts = [
    "-DAOS_DEBUG=1",
]

llvm_toolchain(
    name = "llvm_toolchain",
    additional_target_compatible_with = {
        "linux-armv7": [
            "@//tools/platforms/hardware:raspberry_pi",
        ],
    },
    conlyopts = {
        "linux-x86_64": llvm_conlyopts,
        "linux-armv7": llvm_conlyopts,
        "linux-aarch64": llvm_conlyopts,
    },
    copts = {
        "linux-x86_64": llvm_copts,
        "linux-armv7": llvm_copts,
        "linux-aarch64": llvm_copts,
    },
    cxxopts = {
        "linux-x86_64": llvm_cxxopts,
        "linux-armv7": llvm_cxxopts,
        "linux-aarch64": llvm_cxxopts,
    },
    dbg_copts = {
        "linux-x86_64": llvm_dbg_copts,
        "linux-armv7": llvm_dbg_copts,
        "linux-aarch64": llvm_dbg_copts,
    },
    fastbuild_copts = {
        "linux-x86_64": llvm_fastbuild_copts,
        "linux-armv7": llvm_fastbuild_copts,
        "linux-aarch64": llvm_fastbuild_copts,
    },
    llvm_version = llvm_version,
    opt_copts = {
        "linux-x86_64": llvm_opt_copts,
        "linux-armv7": llvm_opt_copts,
        "linux-aarch64": llvm_opt_copts,
    },
    standard_libraries = {
        "linux-x86_64": "libstdc++-10",
        "linux-armv7": "libstdc++-10",
        "linux-aarch64": "libstdc++-10",
    },
    static_libstdcxx = False,
    sysroot = {
        "linux-x86_64": "@amd64_debian_sysroot//:sysroot_files",
        "linux-armv7": "@armhf_debian_rootfs//:sysroot_files",
        "linux-aarch64": "@arm64_debian_rootfs//:sysroot_files",
    },
    target_toolchain_roots = {
        "linux-x86_64": "@llvm_k8//",
        "linux-armv7": "@llvm_armv7//",
        "linux-aarch64": "@llvm_aarch64//",
    },
    toolchain_roots = {
        "linux-x86_64": "@llvm_k8//",
    },
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")

llvm_register_toolchains()

register_toolchains(
    "//tools/cpp:cc-toolchain-roborio",
    "//tools/cpp:cc-toolchain-cortex-m4f",
    "//tools/cpp:cc-toolchain-rp2040",
    # Find a good way to select between these two M4F toolchains.
    #"//tools/cpp:cc-toolchain-cortex-m4f-k22",
    "//tools/python:python_toolchain",
    "//tools/go:noop_go_toolchain",
    "//tools/rust:rust-toolchain-roborio",
    "//tools/rust:noop_rust_toolchain",
    "//tools/ts:noop_node_toolchain",
)

load("//tools/ci:repo_defs.bzl", "ci_configure")

ci_configure(name = "ci_configure")

http_archive(
    name = "platforms",
    sha256 = "2c8d8347427e6bb0ba7cf9f933c08fe2be2b62ff2454546ad852f7bf267aad87",
    strip_prefix = "platforms-e658a6af526089406d0057160542597501ba65d7",
    url = "https://github.com/bazelbuild/platforms/archive/e658a6af526089406d0057160542597501ba65d7.zip",
)

http_archive(
    name = "bazel_skylib",
    sha256 = "1c531376ac7e5a180e0237938a2536de0c54d93f5c278634818e0efc952dd56c",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz",
        "https://github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz",
    ],
)

http_archive(
    name = "python_repo",
    build_file = "@//debian:python.BUILD",
    sha256 = "048c51872f9c3853ae4e961c710533f477194a3f170b454e8d582f32a83e90f5",
    url = "https://www.frc971.org/Build-Dependencies/python-6.tar.gz",
)

http_archive(
    name = "com_github_stevengj_nlopt",
    build_file = "@//debian:nlopt.BUILD",
    patch_args = ["-p1"],
    patches = ["//debian:nlopt.patch"],
    sha256 = "2d65815b21c30813499fe19c63947f7da56b10c0d4a459dce05417899b43e461",
    strip_prefix = "nlopt-496be736b8b249273838b891f4c8ca3669551127",
    url = "https://www.frc971.org/Build-Dependencies/nlopt-496be736b8b249273838b891f4c8ca3669551127.zip",
)

local_repository(
    name = "com_google_absl",
    path = "third_party/abseil",
)

local_repository(
    name = "org_tuxfamily_eigen",
    path = "third_party/eigen",
)

local_repository(
    name = "com_github_rawrtc_re",
    path = "third_party/rawrtc/re",
)

local_repository(
    name = "com_github_rawrtc_rew",
    path = "third_party/rawrtc/rew",
)

local_repository(
    name = "com_github_rawrtc_usrsctp",
    path = "third_party/rawrtc/usrsctp",
)

local_repository(
    name = "com_github_rawrtc_rawrtc_common",
    path = "third_party/rawrtc/rawrtc-common",
)

local_repository(
    name = "com_github_rawrtc_rawrtc_data_channel",
    path = "third_party/rawrtc/rawrtc-data-channel",
)

local_repository(
    name = "com_github_rawrtc_rawrtc",
    path = "third_party/rawrtc/rawrtc",
)

http_archive(
    name = "boringssl",
    patch_args = ["-p1"],
    patches = ["//debian:boringssl.patch"],
    sha256 = "bcab08a22c28f5322316542aa2c3a9ef0a9f9fde9be22d489cee574867b24675",
    strip_prefix = "boringssl-613fe9dbe74b58d6aaaf0d22fe57dccd964c7413",
    urls = ["https://www.frc971.org/Build-Dependencies/boringssl-613fe9dbe74b58d6aaaf0d22fe57dccd964c7413.zip"],
)

# C++ rules for Bazel.
http_archive(
    name = "rules_cc",
    sha256 = "ed36cc7a6f46b7c28ab4009db4a37e350e1ba367446b0886bcc9cdc1df92752e",
    strip_prefix = "rules_cc-608c7b605fb844a20e96a3eddc9b49ad2542adab",
    urls = [
        "https://www.frc971.org/Build-Dependencies/rules_cc-608c7b605fb844a20e96a3eddc9b49ad2542adab.zip",
    ],
)

# Note that rules_python is currently only imported to make googletest happy.
# TODO: add frc971.org URL
http_archive(
    name = "rules_python",
    sha256 = "895fa3b03898d7708eb50ed34dcfb71c07866433df6912a6ff4f4fb473048f99",
    strip_prefix = "rules_python-2b1d6beb4d5d8f59d629597e30e9aa519182d9a9",
    url = "https://github.com/bazelbuild/rules_python/archive/2b1d6beb4d5d8f59d629597e30e9aa519182d9a9.tar.gz",
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

# TODO(austin): https://github.com/wpilibsuite/roborio-toolchain/releases/tag/v2022-1
http_archive(
    name = "arm_frc_linux_gnueabi_repo",
    build_file = "@//tools/cpp/arm-frc-linux-gnueabi:arm-frc-linux-gnueabi.BUILD",
    patches = ["//debian:fts.patch"],
    sha256 = "043a5b047c2af9cf80d146d8327b588264c98a01e0f3f41e3564dd2bbbc95c0e",
    strip_prefix = "frc2020/roborio/",
    url = "https://www.frc971.org/Build-Dependencies/FRC-2020-Linux-Toolchain-7.3.0.tar.gz",
)

# The main partition from https://downloads.raspberrypi.org/raspios_lite_armhf/images/raspios_lite_armhf-2021-11-08/2021-10-30-raspios-bullseye-armhf-lite.zip.sig
# The following files and folders are removed to make bazel happy with it:
#   usr/share/ca-certificates
#   lib/systemd/system/system-systemd\\x2dcryptsetup.slice
http_archive(
    name = "armhf_debian_rootfs",
    build_file = "@//:compilers/debian_rootfs.BUILD",
    sha256 = "734f26a0cfc943cc3cae88412536186adfc4ed148cc167e6ffb298497c686280",
    url = "https://www.frc971.org/Build-Dependencies/2021-10-30-raspios-bullseye-armhf-lite_rootfs.tar.bz2",
)

# The main partition from https://downloads.raspberrypi.org/raspios_lite_armhf/images/raspios_lite_armhf-2021-11-08/2021-10-30-raspios-bullseye-armhf-lite.zip.sig
# The following files and folders are removed to make bazel happy with it:
#   usr/share/ca-certificates
#   lib/systemd/system/system-systemd\\x2dcryptsetup.slice
http_archive(
    name = "arm64_debian_rootfs",
    build_file = "@//:compilers/debian_rootfs.BUILD",
    sha256 = "7e6ad432fec0a36f8b66c3fc2ab8795ea446e61f7dce7a206b55602677cf0904",
    url = "https://www.frc971.org/Build-Dependencies/2021-10-30-raspios-bullseye-arm64-lite_rootfs.tar.bz2",
)

# Created with:
#   `debootstrap buster buster_sysroot`
# and then chrooting in and running:
#   apt install libc6-dev libstdc++-7-dev
# removing the apt cache,
# and then tarring up the result
http_archive(
    name = "amd64_debian_sysroot",
    build_file = "@//:compilers/debian_rootfs.BUILD",
    sha256 = "5e10f4cac85a98a39da1716b218bc05fff4666c61cc471a7df27876710bc86d2",
    url = "https://www.frc971.org/Build-Dependencies/2022-01-06-debian-bullseye_rootfs.tar.bz2",
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
# git fetch https://github.com/wpilibsuite/ni-libraries main
# git archive --output=allwpilib_ni-libraries_776db4e8aed31a651fa2f590e7468c69b384b42a.tar.gz --format=tar.gz 776db4e8aed31a651fa2f590e7468c69b384b42a
http_archive(
    name = "allwpilib_ni_libraries",
    build_file = "@//debian:ni-libraries.BUILD",
    sha256 = "525b9d777cd00091b9095893c2e4fd0a70c7609c76db161161d407769ad4ba74",
    url = "https://www.frc971.org/Build-Dependencies/allwpilib_ni-libraries_776db4e8aed31a651fa2f590e7468c69b384b42a.tar.gz",
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
    name = "postgresql_amd64",
    build_file = "@//debian:postgresql_amd64.BUILD",
    sha256 = "2b8bb77deaf58f798c296ce31ee7a32781395d55e05dcddc8a7da7e827f38d7f",
    url = "https://www.frc971.org/Build-Dependencies/postgresql_amd64.tar.gz",
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
    name = "apache2",
    build_file = "@//debian:apache2.BUILD",
    sha256 = "98b0ad6d911751ba0aa486429e6278f995e7bbabd928c7d3d44c888fa2bf371b",
    url = "https://www.frc971.org/Build-Dependencies/apache2.tar.gz",
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
    sha256 = "71d1512f1a9a3c90496f0ef3adcd46c4e5e4da4310d7cbb6b0da01a07e5e76e8",
    url = "https://www.frc971.org/Build-Dependencies/matplotlib-6.tar.gz",
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
    sha256 = "36db18fc2b2c9012312b5d1cdc3d392d7e9756040f759ea50cb623fea29ae817",
    url = "https://www.frc971.org/Build-Dependencies/python_gtk-4.tar.gz",
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

# Java9 JDK.
remote_java_repository(
    name = "openjdk_linux_archive",
    exec_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    prefix = "openjdk",
    sha256 = "f27cb933de4f9e7fe9a703486cf44c84bc8e9f138be0c270c9e5716a32367e87",
    strip_prefix = "zulu9.0.7.1-jdk9.0.7-linux_x64-allmodules",
    urls = [
        "https://www.frc971.org/Build-Dependencies/zulu9.0.7.1-jdk9.0.7-linux_x64-allmodules.tar.gz",
    ],
    version = "9",
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

http_archive(
    name = "ctre_phoenix_api_cpp_headers",
    build_file_content = """
cc_library(
    name = 'api-cpp',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/phoenix/**/*.h']),
)
""",
    sha256 = "ea4131d1809bc8ccbd72b15cc7a65bd6ebb89a65019afc6a336e2c92d91ec824",
    urls = [
        "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/api-cpp/5.21.1/api-cpp-5.21.1-headers.zip",
    ],
)

http_archive(
    name = "ctre_phoenix_api_cpp_athena",
    build_file_content = """
filegroup(
    name = 'shared_libraries',
    srcs = [
        'linux/athena/shared/libCTRE_Phoenix.so',
    ],
    visibility = ['//visibility:public'],
)

cc_library(
    name = 'api-cpp',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/shared/libCTRE_Phoenix.so'],
    target_compatible_with = ['@//tools/platforms/hardware:roborio'],
)
""",
    sha256 = "328130012a0fc1050c3ff09f30a2adf5106d15accc3d850b744fa60ec635a462",
    urls = [
        "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/api-cpp/5.21.1/api-cpp-5.21.1-linuxathena.zip",
    ],
)

http_archive(
    name = "ctre_phoenix_cci_headers",
    build_file_content = """
cc_library(
    name = 'cci',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/phoenix/**/*.h']),
)
""",
    sha256 = "b3332885c6afe082f9f67c2335086e89f705b6ac6c5101188616f81c58d3e49a",
    urls = [
        "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/cci/5.21.1/cci-5.21.1-headers.zip",
    ],
)

http_archive(
    name = "ctre_phoenix_cci_athena",
    build_file_content = """
filegroup(
    name = 'shared_libraries',
    srcs = [
        'linux/athena/shared/libCTRE_PhoenixCCI.so',
    ],
    visibility = ['//visibility:public'],
)

cc_library(
    name = 'cci',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/shared/libCTRE_PhoenixCCI.so'],
    target_compatible_with = ['@//tools/platforms/hardware:roborio'],
)
""",
    sha256 = "94812541734d7905774d97e10a97e9c79b5c37cba60d9b6b2d6e4bf3bbabc2fb",
    urls = [
        "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/cci/5.21.1/cci-5.21.1-linuxathena.zip",
    ],
)

http_archive(
    name = "build_bazel_rules_nodejs",
    sha256 = "cfc289523cf1594598215901154a6c2515e8bf3671fd708264a6f6aefe02bf39",
    urls = ["https://github.com/bazelbuild/rules_nodejs/releases/download/4.4.6/rules_nodejs-4.4.6.tar.gz"],
)

load("@build_bazel_rules_nodejs//:index.bzl", "node_repositories", "yarn_install")

node_repositories()

# Setup Bazel managed npm dependencies with the `yarn_install` rule.

# To run yarn by hand, use:
#  bazel run @nodejs_linux_amd64//:bin/yarn -- list
# I'm sure there is a better path, but that works...
yarn_install(
    name = "npm",
    frozen_lockfile = True,
    package_json = "//:package.json",
    patch_args = ["-p1"],
    post_install_patches = [
        "//third_party:npm/@bazel/protractor/bazel-protractor.patch",
    ],
    symlink_node_modules = False,
    yarn_lock = "//:yarn.lock",
)

load("@build_bazel_rules_nodejs//toolchains/esbuild:esbuild_repositories.bzl", "esbuild_repositories")

esbuild_repositories(npm_repository = "npm")

http_archive(
    name = "io_bazel_rules_webtesting",
    patch_args = ["-p1"],
    patches = [
        "@//third_party:rules_webtesting/rules_webtesting.patch",
    ],
    sha256 = "e9abb7658b6a129740c0b3ef6f5a2370864e102a5ba5ffca2cea565829ed825a",
    urls = ["https://github.com/bazelbuild/rules_webtesting/releases/download/0.3.5/rules_webtesting.tar.gz"],
)

http_archive(
    name = "rules_rust",
    sha256 = "531bdd470728b61ce41cf7604dc4f9a115983e455d46ac1d0c1632f613ab9fc3",
    strip_prefix = "rules_rust-d8238877c0e552639d3e057aadd6bfcf37592408",
    urls = [
        # `main` branch as of 2021-08-23
        "https://github.com/bazelbuild/rules_rust/archive/d8238877c0e552639d3e057aadd6bfcf37592408.tar.gz",
    ],
)

load("@rules_rust//rust:repositories.bzl", "rust_repository_set")

rust_repository_set(
    name = "rust",
    edition = "2021",
    exec_triple = "x86_64-unknown-linux-gnu",
    extra_target_triples = [
        "arm-unknown-linux-gnueabi",
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
    version = "1.56.1",
)

load("@io_bazel_rules_webtesting//web:repositories.bzl", "web_test_repositories")

web_test_repositories()

load("@io_bazel_rules_webtesting//web/versioned:browsers-0.3.3.bzl", "browser_repositories")

browser_repositories(chromium = True)

# Flatbuffers
local_repository(
    name = "com_github_google_flatbuffers",
    path = "third_party/flatbuffers",
)

http_file(
    name = "sample_logfile",
    downloaded_file_path = "log.fbs",
    sha256 = "45d1d19fb82786c476d3f21a8d62742abaeeedf4c16a00ec37ae350dcb61f1fc",
    urls = ["https://www.frc971.org/Build-Dependencies/small_sample_logfile2.fbs"],
)

http_archive(
    name = "drivetrain_replay",
    build_file_content = """
filegroup(
    name = "drivetrain_replay",
    srcs = glob(["**/*.bfbs"]),
    visibility = ["//visibility:public"],
)
    """,
    sha256 = "115dcd2fe005cb9cad3325707aa7f4466390c43a08555edf331c06c108bdf692",
    url = "https://www.frc971.org/Build-Dependencies/2021-03-20_drivetrain_spin_wheels.tar.gz",
)

http_archive(
    name = "superstructure_replay",
    build_file_content = """
filegroup(
    name = "superstructure_replay",
    srcs = glob(["**/*.bfbs"]),
    visibility = ["//visibility:public"],
)
    """,
    sha256 = "2b9a3ecc83f2aba89a1909ae38fe51e6718a5b4d0e7c131846dfb2845df9cd19",
    url = "https://www.frc971.org/Build-Dependencies/2021-10-03_superstructure_shoot_balls.tar.gz",
)

# OpenCV arm64 (for raspberry pi)
http_archive(
    name = "opencv_arm64",
    build_file = "@//debian:opencv.BUILD",
    sha256 = "d284fae46ca710cf24c81ff7ace34929773466bff38f365a80371bea3b36a2ed",
    url = "https://www.frc971.org/Build-Dependencies/opencv_arm64.tar.gz",
)

# OpenCV armhf (for raspberry pi)
http_archive(
    name = "opencv_armhf",
    build_file = "@//debian:opencv.BUILD",
    sha256 = "064165507bad1afa8f7b22961a9a9067b243abc70064d713d26b37bc8dc2bf56",
    url = "https://www.frc971.org/Build-Dependencies/opencv_armhf_v4.tar.gz",
)

http_archive(
    name = "opencv_k8",
    build_file = "@//debian:opencv.BUILD",
    sha256 = "b730f5d0c8eb59411157b4e84bfdddd3a6fceed10b842716d9d1b74ad322ea14",
    url = "https://www.frc971.org/Build-Dependencies/opencv_amd64_v3.tar.gz",
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
# https://github.com/halide/Halide/releases/download/v8.0.0/halide-arm64-linux-64-trunk-65c26cba6a3eca2d08a0bccf113ca28746012cc3.tgz
# which is "Halide 8.0.0" at https://github.com/halide/Halide/releases.
# The "2019/08/27" release was renamed as per the release notes:
# https://github.com/halide/Halide/releases/tag/v8.0.0
http_archive(
    name = "halide_arm64",
    build_file = "@//debian:halide.BUILD",
    sha256 = "97b3e54565cd9df52abdd6452f3720ffd38861524154d74ae3d20dc949ed2a63",
    strip_prefix = "halide/",
    url = "https://www.frc971.org/Build-Dependencies/halide-arm64-linux-64-trunk-65c26cba6a3eca2d08a0bccf113ca28746012cc3.tgz",
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
# https://files.pythonhosted.org/packages/0f/13/192104516c4a3d92dc6b5e106ffcfbf0fe35f3c4faa49650205ff652af72/opencv_python-4.5.1.48-cp37-cp37m-manylinux2014_x86_64.whl
http_archive(
    name = "opencv_contrib_nonfree_amd64",
    build_file = "@//debian:opencv_python.BUILD",
    sha256 = "a1dfa0486db367594510c0c799ec7481247dc86e651b69008806d875ab731471",
    type = "zip",
    url = "https://www.frc971.org/Build-Dependencies/opencv_python-4.5.1.48-cp39-cp39-manylinux2014_x86_64.whl",
)

http_archive(
    name = "osqp_amd64",
    build_file = "@//debian:osqp_python.BUILD",
    sha256 = "8003fc363f707daa46fef3af548e6a580372154d6cd49a7bf2f569ba5f807d15",
    type = "zip",
    url = "https://files.pythonhosted.org/packages/3f/e2/f1c40e890f00f8a566bc2481d0f215e52def3dfe8eea6b8ad4cc2d3cbca2/osqp-0.6.2.post5-cp39-cp39-manylinux_2_5_x86_64.manylinux1_x86_64.manylinux_2_17_x86_64.manylinux2014_x86_64.whl",
)

http_archive(
    name = "qdldl_amd64",
    build_file = "@//debian:qdldl_python.BUILD",
    sha256 = "2c09f4b1a1c6f3a0579af004443417e084491e7c844ff9fb73170bb5d43f70b5",
    type = "zip",
    url = "https://files.pythonhosted.org/packages/9e/26/ccb4f065b40c1e9ff35ee66970d4fa97dd2fe221b846da2110eb8cd6c3f4/qdldl-0.1.5.post0-cp39-cp39-manylinux2014_x86_64.whl",
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

local_repository(
    name = "snappy",
    path = "third_party/snappy",
)

http_archive(
    name = "io_bazel_rules_go",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@//third_party:rules_go/0001-Disable-warnings-for-external-repositories.patch",
    ],
    sha256 = "2b1641428dff9018f9e85c0384f03ec6c10660d935b750e3fa1492a281a53b0f",
    urls = [
        "https://www.frc971.org/Build-Dependencies/rules_go-v0.29.0.zip",
        "https://github.com/bazelbuild/rules_go/releases/download/v0.29.0/rules_go-v0.29.0.zip",
    ],
)

http_archive(
    name = "bazel_gazelle",
    sha256 = "de69a09dc70417580aabf20a28619bb3ef60d038470c7cf8442fafcf627c21cb",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
        "https://github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
    ],
)

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains", "go_rules_dependencies")

go_rules_dependencies()

go_register_toolchains(version = "1.17.1")

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")
load("//:go_deps.bzl", "go_dependencies")
load("//tools/go:mirrored_go_deps.bzl", "mirrored_go_dependencies")

mirrored_go_dependencies()

# gazelle:repository_macro go_deps.bzl%go_dependencies
go_dependencies()

gazelle_dependencies()

http_archive(
    name = "com_github_bazelbuild_buildtools",
    sha256 = "44a6e5acc007e197d45ac3326e7f993f0160af9a58e8777ca7701e00501c0857",
    strip_prefix = "buildtools-4.2.4",
    urls = [
        "https://github.com/bazelbuild/buildtools/archive/refs/tags/4.2.4.tar.gz",
    ],
)

http_archive(
    name = "rules_pkg",
    patch_args = ["-p1"],
    patches = [
        "//third_party:rules_pkg/0001-Fix-tree-artifacts.patch",
    ],
    sha256 = "62eeb544ff1ef41d786e329e1536c1d541bb9bcad27ae984d57f18f314018e66",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_pkg/releases/download/0.6.0/rules_pkg-0.6.0.tar.gz",
        "https://github.com/bazelbuild/rules_pkg/releases/download/0.6.0/rules_pkg-0.6.0.tar.gz",
    ],
)

load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")

rules_pkg_dependencies()

http_archive(
    name = "libtinfo5",
    build_file_content = """
exports_files(
    [
        'lib/x86_64-linux-gnu/libtinfo.so.5',
        'lib/x86_64-linux-gnu/libtinfo.so.5.9',
    ],
    ["//visibility:public"],
)
""",
    patch_cmds = ["touch lib/x86_64-linux-gnu/BUILD"],
    sha256 = "059e14f77dce365c57b96284aae98c892f61e269b3fbb7d07714b7135c2e5617",
    urls = ["https://www.frc971.org/Build-Dependencies/libtinfo5_amd64.tar.gz"],
)
