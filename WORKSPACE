workspace(name = "org_frc971")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/jdk:remote_java_repository.bzl", "remote_java_repository")
load("//tools/ci:repo_defs.bzl", "ci_configure")

ci_configure(name = "ci_configure")

load("@ci_configure//:ci.bzl", "RUNNING_IN_CI")

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
    name = "rules_python",
    patch_args = ["-p1"],
    patches = [
        "//third_party:rules_python/0001-Support-overriding-individual-packages.patch",
        "//third_party:rules_python/0002-Allow-user-to-patch-wheels.patch",
    ],
    sha256 = "497ca47374f48c8b067d786b512ac10a276211810f4a580178ee9b9ad139323a",
    strip_prefix = "rules_python-0.16.1",
    url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.16.1.tar.gz",
)

load("@rules_python//python:repositories.bzl", "python_register_toolchains")

python_register_toolchains(
    name = "python3_9",
    python_version = "3.9",
    register_toolchains = False,
)

load("@python3_9//:defs.bzl", python_interpreter = "interpreter")
load("@rules_python//python:pip.bzl", "pip_parse")
load("//tools/python:package_annotations.bzl", PYTHON_ANNOTATIONS = "ANNOTATIONS")

pip_parse(
    name = "pip_deps",
    annotations = PYTHON_ANNOTATIONS,
    enable_implicit_namespace_pkgs = True,
    overrides = "//tools/python:whl_overrides.json",
    patch_spec = "//tools/python:patches.json",
    python_interpreter_target = python_interpreter,
    require_overrides = RUNNING_IN_CI,
    requirements_lock = "//tools/python:requirements.lock.txt",
)

# Load the starlark macro which will define your dependencies.
load("@pip_deps//:requirements.bzl", install_pip_deps = "install_deps")

install_pip_deps()

load("//tools/python:repo_defs.bzl", "pip_configure")

pip_configure(
    name = "pip",
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
    "//debian:arm_frc_gnueabi_deps.bzl",
    arm_frc_gnueabi_deps_debs = "files",
)
load(
    "//debian:gtk_runtime.bzl",
    gtk_runtime_debs = "files",
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
    "//debian:gstreamer_arm64.bzl",
    gstreamer_arm64_debs = "files",
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
load(
    "//debian:libtinfo5_arm64.bzl",
    libtinfo5_arm64_debs = "files",
)
load("//debian:packages.bzl", "generate_repositories_for_debs")

generate_repositories_for_debs(rsync_debs)

generate_repositories_for_debs(ssh_debs)

generate_repositories_for_debs(apache2_debs)

generate_repositories_for_debs(postgresql_amd64_debs)

generate_repositories_for_debs(patch_debs)

generate_repositories_for_debs(pandoc_debs)

generate_repositories_for_debs(libusb_debs)

generate_repositories_for_debs(mingw_compiler_debs)

generate_repositories_for_debs(patchelf_debs)

generate_repositories_for_debs(arm_frc_gnueabi_deps_debs)

generate_repositories_for_debs(gtk_runtime_debs)

generate_repositories_for_debs(opencv_arm64_debs)

generate_repositories_for_debs(opencv_armhf_debs)

generate_repositories_for_debs(opencv_amd64_debs)

generate_repositories_for_debs(
    gstreamer_amd64_debs,
    base_url = "https://www.frc971.org/Build-Dependencies/gstreamer_bullseye_amd64_deps",
)

generate_repositories_for_debs(gstreamer_armhf_debs)

generate_repositories_for_debs(
    gstreamer_arm64_debs,
    base_url = "https://www.frc971.org/Build-Dependencies/gstreamer_bullseye_arm64_deps",
)

generate_repositories_for_debs(m4_debs)

generate_repositories_for_debs(lzma_amd64_debs)

generate_repositories_for_debs(lzma_arm64_debs)

generate_repositories_for_debs(libtinfo5_amd64_debs)

generate_repositories_for_debs(libtinfo5_arm64_debs)

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
        "linux-aarch64": "@llvm_aarch64//",
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
    "//tools/rust:rust-toolchain-x86",
    "//tools/rust:rust-toolchain-armv7",
    "//tools/rust:rust-toolchain-arm64",
    # TODO(Brian): Make this work. See the comment on
    # //tools/platforms:linux_roborio for details.
    #"//tools/rust:rust-toolchain-roborio",
    "//tools/rust:noop_rust_toolchain",
    "//tools/ts:noop_node_toolchain",
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

local_repository(
    name = "com_github_gflags_gflags",
    path = "third_party/gflags",
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

# For protobuf. Don't use these.
bind(
    name = "six",
    actual = "@pip//six",
)

bind(
    name = "gtest",
    actual = "@com_google_googletest//:gtest",
)

bind(
    name = "gtest_main",
    actual = "@com_google_googletest//:gtest_main",
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
    name = "gtk_runtime",
    build_file = "@//debian:gtk_runtime.BUILD",
    sha256 = "5a6014d1783363be6bc95843d03bbb6513e650eaea60be2b1a4c65bf21981f9b",
    url = "https://www.frc971.org/Build-Dependencies/gtk_runtime-4.tar.gz",
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

# Java11 JDK.
remote_java_repository(
    name = "openjdk_linux_archive",
    prefix = "openjdk",
    sha256 = "60e65d32e38876f81ddb623e87ac26c820465b637e263e8bed1acdecb4ca9be2",
    strip_prefix = "zulu11.54.25-ca-jdk11.0.14.1-linux_x64",
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    urls = [
        "https://www.frc971.org/Build-Dependencies/zulu11.54.25-ca-jdk11.0.14.1-linux_x64.tar.gz",
    ],
    version = "11",
)

remote_java_repository(
    name = "openjdk_linux_archive_aarch64",
    prefix = "openjdk",
    sha256 = "b0fb0bc303bb05b5042ef3d0939b9489f4a49a13a2d1c8f03c5d8ab23099454d",
    strip_prefix = "zulu11.54.25-ca-jdk11.0.14.1-linux_aarch64",
    target_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    urls = [
        "https://www.frc971.org/Build-Dependencies/zulu11.54.25-ca-jdk11.0.14.1-linux_aarch64.tar.gz",
    ],
    version = "11",
)

local_repository(
    name = "com_google_protobuf",
    path = "third_party/protobuf",
)

local_repository(
    name = "com_github_google_glog",
    path = "third_party/google-glog",
)

http_archive(
    name = "com_google_googletest",
    patch_args = ["-p1"],
    patches = ["//third_party/googletest:googletest.patch"],
    sha256 = "5c6d595243de011f8ce8cc68050cc0490726786e80cdfd6555ebfe148d920407",
    strip_prefix = "googletest-356fc301251378e0f6fa6aa794d73714202887ac",
    urls = ["https://github.com/google/googletest/archive/356fc301251378e0f6fa6aa794d73714202887ac.zip"],
)

# External dependency: Google Benchmark; has no Bazel build.
http_archive(
    name = "com_github_google_benchmark",
    patch_args = ["-p1"],
    patches = ["//third_party/google-benchmark:benchmark.patch"],
    sha256 = "6430e4092653380d9dc4ccb45a1e2dc9259d581f4866dc0759713126056bc1d7",
    strip_prefix = "benchmark-1.7.1",
    urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.7.1.tar.gz"],
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
    name = "rules_rust_tinyjson",
    build_file = "@rules_rust//util/process_wrapper:BUILD.tinyjson.bazel",
    sha256 = "1a8304da9f9370f6a6f9020b7903b044aa9ce3470f300a1fba5bc77c78145a16",
    strip_prefix = "tinyjson-2.3.0",
    type = "tar.gz",
    url = "https://crates.io/api/v1/crates/tinyjson/2.3.0/download",
)

local_repository(
    name = "rules_rust",
    path = "third_party/rules_rust",
)

load("@rules_rust//rust:repositories.bzl", "rust_repository_set")

rust_repository_set(
    name = "rust",
    allocator_library = "@//tools/rust:forward_allocator",
    edition = "2021",
    exec_triple = "x86_64-unknown-linux-gnu",
    extra_target_triples = [
        "arm-unknown-linux-gnueabi",
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
    register_toolchain = False,
    rustfmt_version = "1.62.0",
    version = "1.62.0",
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

http_archive(
    name = "halide_k8",
    build_file = "@//debian:halide.BUILD",
    sha256 = "be3bdd067acb9ee0d37d0830821113cd69174bee46da466a836d8829fef7cf91",
    strip_prefix = "Halide-14.0.0-x86-64-linux/",
    url = "https://github.com/halide/Halide/releases/download/v14.0.0/Halide-14.0.0-x86-64-linux-6b9ed2afd1d6d0badf04986602c943e287d44e46.tar.gz",
)

http_archive(
    name = "halide_arm64",
    build_file = "@//debian:halide.BUILD",
    sha256 = "cdd42411bcbba682f73d7db0af69837c4857ee90f1727c6feb37fc9a98132385",
    strip_prefix = "Halide-14.0.0-arm-64-linux/",
    url = "https://github.com/halide/Halide/releases/download/v14.0.0/Halide-14.0.0-arm-64-linux-6b9ed2afd1d6d0badf04986602c943e287d44e46.tar.gz",
)

http_archive(
    name = "halide_armhf",
    build_file = "@//debian:halide.BUILD",
    sha256 = "6b3fe3396391b57990a2c41d8dcea74b0734d1b2a0fd42fe0858d954aa45df2b",
    strip_prefix = "Halide-14.0.0-arm-32-linux/",
    url = "https://github.com/halide/Halide/releases/download/v14.0.0/Halide-14.0.0-arm-32-linux-6b9ed2afd1d6d0badf04986602c943e287d44e46.tar.gz",
)

http_archive(
    name = "gstreamer_k8",
    build_file = "@//debian:gstreamer.BUILD",
    sha256 = "d4994261a432c188716f0bdf30fc3f0dff6727319d9c58e7156e2b3ed5105248",
    url = "https://www.frc971.org/Build-Dependencies/gstreamer_1.20.1-1~bpo11+1_amd64.tar.gz",
)

http_archive(
    name = "gstreamer_armhf",
    build_file = "@//debian:gstreamer.BUILD",
    sha256 = "c5ac4c604952c274a50636e244f0d091bd1de302032446f24f0e9e03ae9c76f7",
    url = "https://www.frc971.org/Build-Dependencies/gstreamer_armhf.tar.gz",
)

http_archive(
    name = "gstreamer_arm64",
    build_file = "@//debian:gstreamer.BUILD",
    sha256 = "42b414c565ffdbae3d2d7796a66da9de42a650de757fa6554fd624f0cc3aaa9b",
    url = "https://www.frc971.org/Build-Dependencies/gstreamer_1.20.1-1~bpo11+1_arm64.tar.gz",
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
    sha256 = "6fa0ad579b78bd41a0133024c34063b140442dd2ad4201fd2bf4c55229e7c13f",
    urls = ["https://www.frc971.org/Build-Dependencies/lzma_amd64-2.tar.gz"],
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
    sha256 = "b4ab9fd7cf3bfdb9e3fc67ac4a3c84db7f7e3c48431ccfc6e6e210f5829d17c9",
    urls = ["https://www.frc971.org/Build-Dependencies/lzma_arm64-2.tar.gz"],
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

go_register_toolchains(version = "1.18")

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
    name = "libtinfo5_amd64",
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

http_archive(
    name = "libtinfo5_arm64",
    build_file_content = """
exports_files(
    [
        'lib/aarch64-linux-gnu/libtinfo.so.5',
        'lib/aarch64-linux-gnu/libtinfo.so.5.9',
    ],
    ["//visibility:public"],
)
""",
    patch_cmds = ["touch lib/aarch64-linux-gnu/BUILD"],
    sha256 = "df4ea5194c80df8d1f5f6ed68b47ce9dbf78aa8cdebbc61cf00654d9075f8e3c",
    urls = ["https://www.frc971.org/Build-Dependencies/libtinfo5_arm64.tar.gz"],
)

local_repository(
    name = "com_github_nlohmann_json",
    path = "third_party/com_github_nlohmann_json",
)

# https://curl.haxx.se/download/curl-7.69.1.tar.gz
http_archive(
    name = "com_github_curl_curl",
    build_file = "//debian:curl.BUILD",
    sha256 = "01ae0c123dee45b01bbaef94c0bc00ed2aec89cb2ee0fd598e0d302a6b5e0a98",
    strip_prefix = "curl-7.69.1",
    url = "https://www.frc971.org/Build-Dependencies/curl-7.69.1.tar.gz",
)

http_archive(
    # No official name exists.  Names used in our external dependencies include
    # zlib, madler_zlib, com_github_madler_zlib.
    name = "zlib",
    build_file = "//debian:BUILD.zlib.bazel",
    sha256 = "629380c90a77b964d896ed37163f5c3a34f6e6d897311f1df2a7016355c45eff",
    strip_prefix = "zlib-1.2.11",
    urls = [
        "https://github.com/madler/zlib/archive/v1.2.11.tar.gz",
    ],
)

# This one is tricky to get an archive because it has recursive submodules. These semi-automated steps do work though:
# git clone -b version1.9 --recurse-submodules --depth=1 https://github.com/aws/aws-sdk-cpp
# cd aws-sdk-cpp
# echo bsdtar -a -cf aws_sdk-version.tar.gz --ignore-zeros @\<\(git archive HEAD\) $(git submodule foreach --recursive --quiet 'echo @\<\(cd $displaypath \&\& git archive HEAD --prefix=$displaypath/\)')
# Now run the command that printed, and the output will be at aws_sdk-version.tar.gz.
http_archive(
    name = "aws_sdk",
    build_file = "//debian:aws_sdk.BUILD",
    patch_args = ["-p1"],
    patches = ["//debian:aws_sdk.patch"],
    sha256 = "1a2668722e5b5b2608a6ab21c876c1d98b5fd8c7d613129ced9561f5520817dc",
    url = "https://www.frc971.org/Build-Dependencies/aws_sdk-19.0.0-RC1.tar.gz",
)

# Source code of LZ4 (files under lib/) are under BSD 2-Clause.
# The rest of the repository (build information, documentation, etc.) is under GPLv2.
# We only care about the lib/ subfolder anyways, and strip out any other files.
http_archive(
    name = "com_github_lz4_lz4",
    build_file = "//debian:BUILD.lz4.bazel",
    sha256 = "0b0e3aa07c8c063ddf40b082bdf7e37a1562bda40a0ff5272957f3e987e0e54b",
    strip_prefix = "lz4-1.9.4/lib",
    url = "https://github.com/lz4/lz4/archive/refs/tags/v1.9.4.tar.gz",
)

http_file(
    name = "com_github_foxglove_mcap_mcap",
    executable = True,
    sha256 = "ae745dd09cf4c9570c1c038a72630c07b073f0ed4b05983d64108ff748a40d3f",
    urls = ["https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.22/mcap-linux-amd64"],
)

http_archive(
    name = "cargo_raze",
    patches = ["@//third_party/cargo_raze:cargo_raze.patch"],
    sha256 = "08bfc8859ff686ecb55005a3c4a9cf790115de0abdbcc69cf57b15be0745a859",
    strip_prefix = "cargo-raze-0.14.2",
    url = "https://github.com/google/cargo-raze/archive/v0.14.2.tar.gz",
)

http_archive(
    name = "rules_foreign_cc",
    patch_args = ["-p1"],
    patches = ["@//third_party/rules_foreign_cc:backport_d93bd96dc719760c968b54730258ad0a5b10f8fb.patch"],
    sha256 = "69023642d5781c68911beda769f91fcbc8ca48711db935a75da7f6536b65047f",
    strip_prefix = "rules_foreign_cc-0.6.0",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/0.6.0.tar.gz",
)

load("@cargo_raze//:repositories.bzl", "cargo_raze_repositories")

cargo_raze_repositories()

# cargo_raze_transitive_deps wants to do `rust_repositories`, which we do
# manually to get the right platforms. rules_foreign_cc is currently the only
# other thing it has, so just do that manually.
load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()

load("//third_party/cargo:crates.bzl", "raze_fetch_remote_crates")

raze_fetch_remote_crates()

http_archive(
    name = "com_github_zaphoyd_websocketpp",
    build_file = "//third_party/websocketpp:websocketpp.BUILD",
    sha256 = "6ce889d85ecdc2d8fa07408d6787e7352510750daa66b5ad44aacb47bea76755",
    strip_prefix = "websocketpp-0.8.2",
    url = "https://github.com/zaphoyd/websocketpp/archive/refs/tags/0.8.2.tar.gz",
)

http_archive(
    name = "com_github_foxglove_ws-protocol",
    build_file = "//third_party/foxglove_ws_protocol:foxglove_ws_protocol.BUILD",
    patch_args = ["-p1"],
    patches = ["//third_party/foxglove_ws_protocol:foxglove_ws_protocol.patch"],
    sha256 = "3256f09a67419f6556778c443d332f1a4bf53ba0e7a464179bf838abffa366ab",
    strip_prefix = "ws-protocol-releases-typescript-ws-protocol-examples-v0.0.6",
    url = "https://github.com/foxglove/ws-protocol/archive/refs/tags/releases/typescript/ws-protocol-examples/v0.0.6.tar.gz",
)

http_archive(
    name = "asio",
    build_file_content = """
cc_library(
    name = "asio",
    hdrs = glob(["include/asio/**/*.hpp", "include/asio/**/*.ipp", "include/asio.hpp"]),
    visibility = ["//visibility:public"],
    defines = ["ASIO_STANDALONE"],
    includes = ["include/"],
)""",
    sha256 = "8976812c24a118600f6fcf071a20606630a69afe4c0abee3b0dea528e682c585",
    strip_prefix = "asio-1.24.0",
    url = "https://downloads.sourceforge.net/project/asio/asio/1.24.0%2520%2528Stable%2529/asio-1.24.0.tar.bz2",
)
