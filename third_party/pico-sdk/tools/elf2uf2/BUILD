cc_binary(
    name = "elf2uf2",
    srcs = [
        "elf.h",
        "main.cpp",
    ],
    copts = [
        "-Wno-format-nonliteral",
        "-Wno-reorder",
        "-Wno-unused-parameter",
        "-Wno-unused-function",
        "-Wno-type-limits",
    ],
    includes = ["."],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/pico-sdk/src/common/boot_uf2",
    ],
)
