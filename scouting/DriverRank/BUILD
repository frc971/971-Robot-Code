load("@rules_pkg//:pkg.bzl", "pkg_deb", "pkg_tar")

filegroup(
    name = "driver_rank_script",
    srcs = [
        "src/DriverRank.jl",
    ],
    visibility = ["//scouting:__subpackages__"],
)

pkg_tar(
    name = "julia_runtime",
    package_dir = "opt/frc971/julia_runtime",
    deps = [
        "@julia//:runtime",
    ],
)

pkg_tar(
    name = "julia_manifest",
    srcs = [
        "Manifest.toml",
        "Project.toml",
        "activate.jl",
    ],
    package_dir = "opt/frc971/julia_manifest",
)

pkg_tar(
    name = "julia_files",
    deps = [
        ":julia_manifest",
        ":julia_runtime",
    ],
)

pkg_deb(
    name = "frc971-scouting-julia",
    architecture = "amd64",
    data = ":julia_files",
    description = "The Julia files for the FRC971 scouting web server.",
    maintainer = "frc971@frc971.org",
    package = "frc971-scouting-julia",
    postinst = "postinst",
    version = "1",
)

py_binary(
    name = "deploy",
    srcs = [
        "deploy.py",
    ],
    args = [
        "--deb",
        "$(location :frc971-scouting-julia)",
    ],
    data = [
        ":frc971-scouting-julia",
    ],
    deps = [
        "//scouting/deploy",
    ],
)
