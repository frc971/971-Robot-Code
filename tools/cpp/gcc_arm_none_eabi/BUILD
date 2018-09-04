package(default_visibility = ["//tools/cpp:__pkg__"])

filegroup(
    name = "gcc",
    srcs = [
        "arm-none-eabi-gcc",
        "@gcc_arm_none_eabi//:gcc",
    ],
)

filegroup(
    name = "ar",
    srcs = [
        "arm-none-eabi-ar",
        "@gcc_arm_none_eabi//:ar",
    ],
)

filegroup(
    name = "ld",
    srcs = [
        "arm-none-eabi-ld",
        "@gcc_arm_none_eabi//:ld",
    ],
)

filegroup(
    name = "nm",
    srcs = [
        "arm-none-eabi-nm",
        "@gcc_arm_none_eabi//:nm",
    ],
)

filegroup(
    name = "objcopy",
    srcs = [
        "arm-none-eabi-objcopy",
        "@gcc_arm_none_eabi//:objcopy",
    ],
)

filegroup(
    name = "objdump",
    srcs = [
        "arm-none-eabi-objdump",
        "@gcc_arm_none_eabi//:objdump",
    ],
)

filegroup(
    name = "strip",
    srcs = [
        "arm-none-eabi-strip",
        "@gcc_arm_none_eabi//:strip",
    ],
)

filegroup(
    name = "as",
    srcs = [
        "arm-none-eabi-as",
        "@gcc_arm_none_eabi//:as",
    ],
)

filegroup(
    name = "tool-wrappers",
    srcs = [
        ":ar",
        ":as",
        ":gcc",
        ":ld",
        ":nm",
        ":objcopy",
        ":objdump",
        ":strip",
    ],
)
