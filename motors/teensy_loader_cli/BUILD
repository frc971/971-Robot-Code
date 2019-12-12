cc_binary(
    name = "teensy_loader_cli",
    srcs = [
        "teensy_loader_cli.c",
    ],
    copts = [
        "-DUSE_LIBUSB",
        "-Wno-format-nonliteral",
    ],
    restricted_to = ["@//tools:k8"],
    deps = [
        "@libusb",
    ],
)
