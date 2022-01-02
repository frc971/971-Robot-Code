load("//third_party/pico-sdk:hex.bzl", "uf2_from_elf")

# TODO(austin): We should be able to either push the linker script into the crosstool, or wrap this in a macro.
cc_binary(
    name = "blink.elf",
    srcs = [
        "blink.c",
        "//third_party/pico-sdk/src/rp2_common/boot_stage2:bs2_default_padded_checksummed.S",
    ],
    additional_linker_inputs = [
        "//third_party/pico-sdk:src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    linkopts = [
        "-Wl,--script=third_party/pico-sdk/src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m0plus",
    ],
    deps = ["//third_party/pico-sdk"],
)

uf2_from_elf(
    name = "blink",
)
