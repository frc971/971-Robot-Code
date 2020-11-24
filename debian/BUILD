package(default_visibility = ["//visibility:public"])

load(
    "//debian:python.bzl",
    python_debs = "files",
)
load(
    "//debian:clang.bzl",
    clang_debs = "files",
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
    ":matplotlib.bzl",
    matplotlib_debs = "files",
)
load(
    ":arm_frc_gnueabi_deps.bzl",
    arm_frc_gnueabi_deps_debs = "files",
)
load(
    ":python_gtk.bzl",
    python_gtk_debs = "files",
)
load(
    ":opencv_armhf.bzl",
    opencv_armhf_debs = "files",
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
    ":gstreamer_armhf.bzl",
    gstreamer_armhf_debs = "files",
)
load(
    ":m4.bzl",
    m4_debs = "files",
)
load(
    ":lzma_amd64.bzl",
    lzma_amd64_debs = "files",
)
load(
    ":lzma_arm64.bzl",
    lzma_arm64_debs = "files",
)
load(":packages.bzl", "download_packages", "generate_deb_tarball")

filegroup(
    name = "matplotlib_patches",
    srcs = [
        "matplotlib_init.patch",
    ],
    visibility = ["@matplotlib_repo//:__pkg__"],
)

filegroup(
    name = "python_shapely_patches",
    srcs = [
        "python_shapely_init.patch",
    ],
    visibility = ["@python_gtk//:__pkg__"],
)

filegroup(
    name = "python_gi_patches",
    srcs = [
        "python_gi_init.patch",
    ],
    visibility = ["@python_gtk//:__pkg__"],
)

filegroup(
    name = "python_geos_patches",
    srcs = [
        "python_geos.patch",
    ],
    visibility = ["@python_gtk//:__pkg__"],
)

py_binary(
    name = "download_packages",
    srcs = [
        "download_packages.py",
    ],
    main = "download_packages.py",
    python_version = "PY3",
    srcs_version = "PY2AND3",
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_python_deps",
    excludes = [
        "libblas.so.3",
        "liblapack.so.3",
    ],
    packages = [
        "python-dev",
        "python-numpy",
        "python-scipy",
        "python3-dev",
        "python3-numpy",
        "python3-scipy",
    ],
    target_compatible_with = ["@platforms//os:linux"],
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

download_packages(
    name = "download_matplotlib_deps",
    excludes = [
        "python-dev",
        "python-numpy",
        "python-scipy",
        "python3-dev",
        "python3-numpy",
        "python3-scipy",
        "x11-common",
        "fonts-freefont",
        "python",
        "libcups2",
    ],
    packages = [
        "python-matplotlib",
        "python-tk",
        "python3-matplotlib",
        "python3-tk",
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

download_packages(
    name = "download_python_gtk_deps",
    excludes = [
        "fonts-freefont",
        "gsettings-backend",
        "libpng-dev",
        "libz-dev",
        "python3-dev",
        "python3",
        "libblas.so.3",
        "liblapack.so.3",
        "libstdc++-dev",
    ],
    packages = [
        "libgtk-3-dev",
        "python3-cairo",
        "python3-gi",
        "python3-gi-cairo",
        "python3-shapely",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_m4",
    packages = [
        "m4",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "python",
    files = python_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "clang_6p0",
    files = clang_debs,
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
    name = "matplotlib",
    files = matplotlib_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "arm_frc_gnueabi_deps",
    files = arm_frc_gnueabi_deps_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "python_gtk",
    files = python_gtk_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

# Starting point for opencv package list below.
download_packages(
    name = "download_opencv",
    packages = [
        "libopencv-calib3d-dev",
        "libopencv-calib3d3.2",
        "libopencv-contrib-dev",
        "libopencv-contrib3.2",
        "libopencv-core-dev",
        "libopencv-core3.2",
        "libopencv-dev",
        "libopencv-features2d-dev",
        "libopencv-features2d3.2",
        "libopencv-flann-dev",
        "libopencv-flann3.2",
        "libopencv-highgui-dev",
        "libopencv-highgui3.2",
        "libopencv-imgcodecs-dev",
        "libopencv-imgcodecs3.2",
        "libopencv-imgproc-dev",
        "libopencv-imgproc3.2",
        "libopencv-ml-dev",
        "libopencv-ml3.2",
        "libopencv-objdetect-dev",
        "libopencv-objdetect3.2",
        "libopencv-photo-dev",
        "libopencv-photo3.2",
        "libopencv-shape-dev",
        "libopencv-shape3.2",
        "libopencv-stitching-dev",
        "libopencv-stitching3.2",
        "libopencv-superres-dev",
        "libopencv-superres3.2",
        "libopencv-ts-dev",
        "libopencv-video-dev",
        "libopencv-video3.2",
        "libopencv-videoio-dev",
        "libopencv-videoio3.2",
        "libopencv-videostab-dev",
        "libopencv-videostab3.2",
        "libopencv-viz-dev",
        "libopencv-viz3.2",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

# This list was generated with download_packages.py on armhf and then
# hand-tweaked to get everything it needs.
generate_deb_tarball(
    name = "opencv_armhf_v3",
    files = opencv_armhf_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

# This was hand-written based on opencv_armhf.bzl.
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
    name = "gstreamer_armhf",
    files = gstreamer_armhf_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "m4",
    files = m4_debs,
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

exports_files([
    "ssh_wrapper.sh",
])
