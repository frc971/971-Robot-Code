load("//tools/go:go_mirrors.bzl", "GO_MIRROR_INFO")
load("@bazel_gazelle//:deps.bzl", "go_repository")
load("@ci_configure//:ci.bzl", "RUNNING_IN_CI")

def maybe_override_go_dep(name, importpath, sum, version):
    """This macro selects between our dependency mirrors and upstream sources.

    We want to use the mirrored version whenever possible. In CI we are required
    to use the mirrored version. For local development we only use the mirrored
    version if it's available. Otherwise we download from the upstream sources.
    """
    if not RUNNING_IN_CI:
        override_go_dep = not (name in GO_MIRROR_INFO and GO_MIRROR_INFO[name]["version"] == version)
    else:
        override_go_dep = False
        if name not in GO_MIRROR_INFO or GO_MIRROR_INFO[name]["version"] != version:
            fail(("The repo {} is not properly mirrored. " +
                  "Please ask someone with mirroring access for help." +
                  "They need to 'bazel run //tools/go:mirror_go_repos -- " +
                  "--ssh_host <software.971spartans.net>'.").format(name))

    # If we want to use the upstream version and we've already imported a
    # mirrored version via mirrored_go_dependencies(), then we override it here
    # by giving the upstream version the same name.
    if override_go_dep:
        go_repository(
            name = name,
            importpath = importpath,
            sum = sum,
            version = version,
        )

def mirrored_go_dependencies():
    """Sets up the Go dependencies we've mirrored."""
    for name in GO_MIRROR_INFO:
        info = GO_MIRROR_INFO[name]
        go_repository(
            name = name,
            strip_prefix = info["strip_prefix"],
            type = "zip",
            urls = [
                "https://www.frc971.org/Build-Dependencies/go_deps/" + info["filename"],
            ],
            sha256 = info["sha256"],
            importpath = info["importpath"],
        )
