cc_binary(
    name = "teensy_loader_cli",
    srcs = [
        "teensy_loader_cli.c",
    ],
    copts = [
        "-DUSE_LIBUSB",
        "-Wno-format-nonliteral",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "@libusb",
    ],
)
