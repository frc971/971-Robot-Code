load("//frc971/downloader:downloader.bzl", "aos_downloader")
load("//tools/build_rules:label.bzl", "expand_label")

def robot_downloader(
        start_binaries,
        name = "download",
        binaries = [],
        data = [],
        dirs = None,
        default_target = None,
        target_compatible_with = ["@//tools/platforms/hardware:roborio"],
        target_type = "roborio"):
    """Sets up the standard robot download targets.

    Attrs:
      start_binaries: A list of cc_binary targets to start on the robot.
      dirs: Passed through to aos_downloader.
      default_target: Passed through to aos_downloader.
    """

    aos_downloader(
        name = name,
        start_srcs = ([
            "//aos:prime_start_binaries",
        ] if target_type == "roborio" else []) + start_binaries,
        srcs = [
            "//aos:prime_binaries",
        ] + binaries + data + ["//frc971/raspi/rootfs:chrt.sh"],
        dirs = dirs,
        target_type = target_type,
        default_target = default_target,
        target_compatible_with = target_compatible_with,
    )

    aos_downloader(
        name = name + "_stripped",
        start_srcs = ([
                         "//aos:prime_start_binaries_stripped",
                     ] if target_type == "roborio" else []) +
                     [expand_label(binary) + ".stripped" for binary in start_binaries],
        srcs = [
            "//aos:prime_binaries_stripped",
        ] + [expand_label(binary) + ".stripped" for binary in binaries] + data + ["//frc971/raspi/rootfs:chrt.sh"],
        dirs = dirs,
        target_type = target_type,
        default_target = default_target,
        target_compatible_with = target_compatible_with,
    )
