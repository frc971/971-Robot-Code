package(default_visibility = ["//visibility:public"])

load(
    "//debian:python.bzl",
    python_debs = "files",
)
load(
    "//debian:clang.bzl",
    clang_debs = "files",
)
load("//debian:packages.bzl", "download_packages", "generate_deb_tarball")

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
        "python3-dev",
        "python3-numpy",
    ],
)

download_packages(
    name = "download_clang_deps",
    excludes = [
        "libblas.so.3",
    ],
    force_includes = [
        "libc6",
        "libc6-dev",
    ],
    packages = [
        "clang-3.6",
        "clang-format-3.6",
        "gcc",
        "gfortran",
    ],
)

generate_deb_tarball(
    name = "python",
    files = python_debs,
)

generate_deb_tarball(
    name = "clang_3p6",
    files = clang_debs,
)
