load("//third_party/pico-sdk:hex.bzl", "bin_from_elf")

cc_library(
    name = "boot_stage2_header",
    hdrs = [
        "asminclude/boot2_helpers/exit_from_boot2.S",
        "asminclude/boot2_helpers/read_flash_sreg.S",
        "asminclude/boot2_helpers/wait_ssi_ready.S",
        "include/boot_stage2/config.h",
    ],
    includes = [
        "asminclude",
        "include",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "boot_stage2",
    hdrs = [
        "boot2_w25q080.S",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":boot_stage2_header",
        # This is probably too much, but the resulting binary is bitwise identical, so what can I complain about.
        "//third_party/pico-sdk",
    ],
)

cc_binary(
    name = "bs2_default.elf",
    srcs = [
        "compile_time_choice.S",
    ],
    additional_linker_inputs = ["boot_stage2.ld"],
    linkopts = [
        "-Wl,--script=third_party/pico-sdk/src/rp2_common/boot_stage2/boot_stage2.ld",
    ],
    deps = ["boot_stage2"],
)

bin_from_elf(
    name = "bs2_default",
)

genrule(
    name = "pad_checksum_python_rename",
    srcs = ["pad_checksum"],
    outs = ["pad_checksum.py"],
    cmd = "cp -f $(SRCS) $(OUTS)",
)

py_binary(
    name = "pad_checksum_python",
    srcs = ["pad_checksum.py"],
    main = "pad_checksum.py",
)

genrule(
    name = "gen_bs2_default_padded_checksummed.S",
    srcs = ["bs2_default.bin"],
    outs = ["bs2_default_padded_checksummed.S"],
    cmd = "python3 $(location :pad_checksum_python) -s 0xffffffff $(SRCS) $(OUTS)",
    tools = [":pad_checksum_python"],
    visibility = ["//visibility:public"],
)
