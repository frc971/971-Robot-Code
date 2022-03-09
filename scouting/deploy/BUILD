load("@rules_pkg//pkg:pkg.bzl", "pkg_deb", "pkg_tar")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files")

pkg_files(
    name = "systemd_files",
    srcs = [
        "scouting.service",
    ],
    prefix = "etc/systemd/system",
)

pkg_tar(
    name = "server_files",
    srcs = [
        "//scouting",
    ],
    include_runfiles = True,
    package_dir = "opt/frc971/scouting_server",
    strip_prefix = ".",
)

pkg_tar(
    name = "deploy_tar",
    srcs = [
        ":systemd_files",
    ],
    deps = [
        ":server_files",
    ],
)

pkg_deb(
    name = "frc971-scouting-server",
    architecture = "amd64",
    data = ":deploy_tar",
    description = "The FRC971 scouting web server.",
    # TODO(phil): What's a good email address for this?
    maintainer = "frc971@frc971.org",
    package = "frc971-scouting-server",
    postinst = "postinst",
    predepends = [
        "systemd",
    ],
    prerm = "prerm",
    version = "1",
)

py_binary(
    name = "deploy",
    srcs = [
        "deploy.py",
    ],
    args = [
        "--deb",
        "$(location :frc971-scouting-server)",
    ],
    data = [
        ":frc971-scouting-server",
    ],
)
