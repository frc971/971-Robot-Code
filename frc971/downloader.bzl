load("//frc971/downloader:downloader.bzl", "aos_downloader")
load("//tools/build_rules:label.bzl", "expand_label")

def robot_downloader(start_binaries, binaries = [], data = [], dirs = None, default_target = None):
    """Sets up the standard robot download targets.

    Attrs:
      start_binaries: A list of cc_binary targets to start on the robot.
      dirs: Passed through to aos_downloader.
      default_target: Passed through to aos_downloader.
    """

    aos_downloader(
        name = "download",
        start_srcs = [
            "//aos:prime_start_binaries",
        ] + start_binaries,
        srcs = [
            "//aos:prime_binaries",
        ] + binaries + data,
        dirs = dirs,
        default_target = default_target,
        restricted_to = ["//tools:roborio"],
    )

    aos_downloader(
        name = "download_stripped",
        start_srcs = [
            "//aos:prime_start_binaries_stripped",
        ] + [expand_label(binary) + ".stripped" for binary in start_binaries],
        srcs = [
            "//aos:prime_binaries_stripped",
        ] + [expand_label(binary) + ".stripped" for binary in binaries] + data,
        dirs = dirs,
        default_target = default_target,
        restricted_to = ["//tools:roborio"],
    )
