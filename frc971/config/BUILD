filegroup(
    name = "rio_robotCommand",
    srcs = ["robotCommand"],
)

sh_binary(
    name = "setup_roborio",
    srcs = ["setup_roborio.sh"],
    data = [
        ":rio_robotCommand",
        ":sctp.ko",
        "@arm_frc_linux_gnueabi_repo//:compiler_pieces",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
