package(default_visibility = ["//visibility:public"])

platform(
    name = "linux_x86",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
        "//tools/platforms/go:has_support",
        "//tools/platforms/rust:has_support",
        "//tools/platforms/nodejs:has_support",
        "//tools/platforms/gpu:nvidia",
    ],
)

platform(
    name = "linux_arm64",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:arm64",
        "//tools/platforms/hardware:raspberry_pi",
        "//tools/platforms/go:lacks_support",
        "//tools/platforms/rust:has_support",
        "//tools/platforms/nodejs:lacks_support",
        "//tools/platforms/gpu:nvidia",
    ],
)

platform(
    name = "linux_roborio",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:armv7",
        "//tools/platforms/hardware:roborio",
        "//tools/platforms/go:lacks_support",
        # TODO(Brian): This almost works, but cxx assumes llvm-ld's linking
        # behavior and doesn't have an easy way to support GNU ld. See
        # https://github.com/dtolnay/cxx/pull/1069 for a bit more explanation.
        # Bazel needs to group multiple things into a single cc_library to
        # handle that, need to figure out how to do that here or switch linkers.
        "//tools/platforms/rust:lacks_support",
        "//tools/platforms/nodejs:lacks_support",
        "//tools/platforms/gpu:none",
    ],
)

platform(
    name = "cortex_m4f",
    constraint_values = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m4f",
        "//tools/platforms/go:lacks_support",
        "//tools/platforms/rust:lacks_support",
        "//tools/platforms/nodejs:lacks_support",
        "//tools/platforms/gpu:none",
    ],
)

platform(
    name = "rp2040",
    constraint_values = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m0plus",
        "//tools/platforms/go:lacks_support",
        "//tools/platforms/rust:lacks_support",
        "//tools/platforms/nodejs:lacks_support",
        "//tools/platforms/gpu:none",
    ],
)

# TODO(phil): Create something for "cortex-m4f-k22" builds.
