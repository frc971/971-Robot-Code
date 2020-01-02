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
load("//debian:packages.bzl", "download_packages", "generate_deb_tarball")

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
    default_python_version = "PY3",
    main = "download_packages.py",
    srcs_version = "PY2AND3",
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
)

download_packages(
    name = "download_patch_deps",
    packages = [
        "patch",
    ],
)

download_packages(
    name = "download_pandoc_deps",
    packages = [
        "pandoc",
    ],
)

download_packages(
    name = "download_libusb_deps",
    packages = [
        "libusb-0.1-4",
        "libusb-1.0-0",
        "libusb-1.0-0-dev",
        "libusb-dev",
    ],
)

download_packages(
    name = "download_mingw_compiler_deps",
    packages = [
        "g++-mingw-w64-x86-64",
    ],
)

download_packages(
    name = "download_patchelf_deps",
    packages = [
        "patchelf",
    ],
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
    ],
    packages = [
        "libgtk-3-dev",
        "python3-cairo",
        "python3-gi",
        "python3-gi-cairo",
        "python3-shapely",
    ],
)

generate_deb_tarball(
    name = "python",
    files = python_debs,
)

generate_deb_tarball(
    name = "clang_6p0",
    files = clang_debs,
)

generate_deb_tarball(
    name = "patch",
    files = patch_debs,
)

generate_deb_tarball(
    name = "rsync",
    files = rsync_debs,
)

generate_deb_tarball(
    name = "ssh",
    files = ssh_debs,
)

generate_deb_tarball(
    name = "pandoc",
    files = pandoc_debs,
)

generate_deb_tarball(
    name = "libusb",
    files = libusb_debs,
)

generate_deb_tarball(
    name = "mingw_compiler",
    files = mingw_compiler_debs,
)

generate_deb_tarball(
    name = "patchelf",
    files = patchelf_debs,
)

generate_deb_tarball(
    name = "matplotlib",
    files = matplotlib_debs,
)

generate_deb_tarball(
    name = "arm_frc_gnueabi_deps",
    files = arm_frc_gnueabi_deps_debs,
)

generate_deb_tarball(
    name = "python_gtk",
    files = python_gtk_debs,
)
