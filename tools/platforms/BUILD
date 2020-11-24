package(default_visibility = ["//visibility:public"])

platform(
    name = "linux_x86",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
)

platform(
    name = "linux_armhf",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:armv7",
        "//tools/platforms/hardware:raspberry_pi",
    ],
)

platform(
    name = "linux_arm64",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:arm64",
    ],
)

platform(
    name = "linux_roborio",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:armv7",
        "//tools/platforms/hardware:roborio",
    ],
)

platform(
    name = "cortex_m4f",
    constraint_values = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m4f",
    ],
)

# TODO(phil): Create something for "cortex-m4f-k22" builds.
