load("//third_party/pico-sdk:hex.bzl", "uf2_from_elf")

cc_binary(
    name = "tof_controller.elf",
    srcs = [
        "tof_controller.cc",
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
    deps = [
        "//third_party/pico-sdk",
        "@vl53l1x_ultra_lite_driver_api//:VL53L1X",
    ],
)

uf2_from_elf(
    name = "tof_controller",
)
