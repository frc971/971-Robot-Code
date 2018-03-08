package(default_visibility = ["//visibility:public"])

load(
    "//debian:python.bzl",
    python_debs = "files",
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

generate_deb_tarball(
    name = "python",
    files = python_debs,
)
