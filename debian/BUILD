load(
    ":apache2.bzl",
    apache2_debs = "files",
)
load(
    ":postgresql_amd64.bzl",
    postgresql_amd64_debs = "files",
)
load(
    ":patch.bzl",
    patch_debs = "files",
)
load(
    ":rsync.bzl",
    rsync_debs = "files",
)
load(
    ":ssh.bzl",
    ssh_debs = "files",
)
load(
    ":pandoc.bzl",
    pandoc_debs = "files",
)
load(
    ":libusb.bzl",
    libusb_debs = "files",
)
load(
    ":mingw_compiler.bzl",
    mingw_compiler_debs = "files",
)
load(
    ":patchelf.bzl",
    patchelf_debs = "files",
)
load(
    ":arm_frc_gnueabi_deps.bzl",
    arm_frc_gnueabi_deps_debs = "files",
)
load(
    ":gtk_runtime.bzl",
    gtk_runtime_debs = "files",
)
load(
    ":opencv_arm64.bzl",
    opencv_arm64_debs = "files",
)
load(
    ":opencv_amd64.bzl",
    opencv_amd64_debs = "files",
)
load(
    ":gstreamer_amd64.bzl",
    gstreamer_amd64_debs = "files",
)
load(
    ":gstreamer_arm64.bzl",
    gstreamer_arm64_debs = "files",
)
load(
    ":lzma_amd64.bzl",
    lzma_amd64_debs = "files",
)
load(
    ":lzma_arm64.bzl",
    lzma_arm64_debs = "files",
)
load(
    ":libtinfo5_amd64.bzl",
    libtinfo5_amd64_debs = "files",
)
load(
    ":libtinfo5_arm64.bzl",
    libtinfo5_arm64_debs = "files",
)
load(
    ":xvfb_amd64.bzl",
    xvfb_amd64_debs = "files",
)
load(":packages.bzl", "download_packages", "generate_deb_tarball")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "download_packages",
    srcs = [
        "download_packages.py",
    ],
    main = "download_packages.py",
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_apache2_packages",
    excludes = [
        "libaprutil1-dbd-mysql",
        "libaprutil1-dbd-odbc",
        "libaprutil1-dbd-pgsql",
        "libaprutil1-dbd-freetds",
        "libstdc++6",
        "lsb-base",
        "debconf",
        "libc6-dev",
    ],
    force_includes = [
        "libaprutil1",
    ],
    packages = [
        "apache2",
    ],
)

download_packages(
    name = "download_gtk_runtime",
    excludes = [
        "libstdc++6",
        "lsb-base",
        "libglib2.0-dev-bin",
        "fonts-freefont",
        "gsettings-backend",
        "libpng-dev",
        "libz-dev",
        "libstdc++-dev",
        "libc6-dev",
    ],
    # Since "libglib2.0-0" pulls in glibc, we need to forcibly remove it again.
    force_excludes = [
        "libc6",
        "libgcc-s1",
    ],
    force_includes = [
        "libglib2.0-0",
    ],
    packages = [
        "gir1.2-gtk-3.0",
        "libgtk-3-dev",
        "librsvg2-common",
    ],
)

download_packages(
    name = "download_clang_deps",
    excludes = [
        "lib32stdc++6",
        "libstdc++6",
    ],
    force_includes = [
        "libc6",
        "libc6-dev",
    ],
    packages = [
        "clang-6.0",
        "clang-format-6.0",
        "gcc",
        "gfortran",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_postgresql_deps",
    excludes = [
        "adduser",
        "debconf",
        "debconf-2.0",
        "libsystemd0",
        "lsb-base",
        "libstdc++6",
        "libc-bin",
        "libc-l10n",
        "netbase",
        "ucf",
        "locales",
        "locales-all",
    ],
    packages = [
        "postgresql",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_patch_deps",
    packages = [
        "patch",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_pandoc_deps",
    packages = [
        "pandoc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_libusb_deps",
    packages = [
        "libusb-0.1-4",
        "libusb-1.0-0",
        "libusb-1.0-0-dev",
        "libusb-dev",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_mingw_compiler_deps",
    packages = [
        "g++-mingw-w64-x86-64",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_patchelf_deps",
    packages = [
        "patchelf",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

# This list was obtained by manually looking at the output from:
# find bazel-out/../../../external/arm_frc_linux_gnueabi_repo/ -executable -type f -exec ldd {} + | sed 's/=>.*//g' | sort -u
download_packages(
    name = "download_arm_frc_gnueabi_deps",
    packages = [
        "libexpat1",
        "libgmp10",
        "libisl10",
        "liblzma5",
        "libmpc3",
        "libmpfr4",
        "libncurses5",
        "libtinfo5",
        "zlib1g",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "apache2",
    files = apache2_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "postgresql_amd64",
    files = postgresql_amd64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "patch",
    files = patch_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "rsync",
    files = rsync_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "ssh_v3",
    files = ssh_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "pandoc",
    files = pandoc_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "libusb",
    files = libusb_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "mingw_compiler",
    files = mingw_compiler_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "patchelf",
    files = patchelf_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "arm_frc_gnueabi_deps",
    files = arm_frc_gnueabi_deps_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "gtk_runtime",
    files = gtk_runtime_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_opencv",
    packages = [
        "libopencv-calib3d-dev",
        "libopencv-calib3d4.5",
        "libopencv-contrib-dev",
        "libopencv-contrib4.5",
        "libopencv-core-dev",
        "libopencv-core4.5",
        "libopencv-dev",
        "libopencv-features2d-dev",
        "libopencv-features2d4.5",
        "libopencv-flann-dev",
        "libopencv-flann4.5",
        "libopencv-highgui-dev",
        "libopencv-highgui4.5",
        "libopencv-imgcodecs-dev",
        "libopencv-imgcodecs4.5",
        "libopencv-imgproc-dev",
        "libopencv-imgproc4.5",
        "libopencv-ml-dev",
        "libopencv-ml4.5",
        "libopencv-objdetect-dev",
        "libopencv-objdetect4.5",
        "libopencv-photo-dev",
        "libopencv-photo4.5",
        "libopencv-shape-dev",
        "libopencv-shape4.5",
        "libopencv-stitching-dev",
        "libopencv-stitching4.5",
        "libopencv-superres-dev",
        "libopencv-superres4.5",
        "libopencv-video-dev",
        "libopencv-video4.5",
        "libopencv-videoio-dev",
        "libopencv-videoio4.5",
        "libopencv-videostab-dev",
        "libopencv-videostab4.5",
        "libopencv-viz-dev",
        "libopencv-viz4.5",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "opencv_arm64",
    files = opencv_arm64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "opencv_amd64",
    files = opencv_amd64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "gstreamer_amd64",
    files = gstreamer_amd64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "gstreamer_arm64",
    files = gstreamer_arm64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_lzma",
    packages = [
        "liblzma-dev",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "lzma_amd64",
    files = lzma_amd64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "lzma_arm64",
    files = lzma_arm64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "libtinfo5_amd64",
    files = libtinfo5_amd64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "libtinfo5_arm64",
    files = libtinfo5_arm64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_xvfb_packages",
    excludes = [
        "libglx-mesa0",
    ],
    packages = [
        "xvfb",
    ],
)

generate_deb_tarball(
    name = "xvfb_amd64",
    files = xvfb_amd64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

exports_files([
    "ssh_wrapper.sh",
    "curl.BUILD",
    "BUILD.zlib.bazel",
])
