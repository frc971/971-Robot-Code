cc_library(
    name = "ni-libraries",
    srcs = [
        "lib/libFRC_NetworkCommunication.so.18.0.0",
        "lib/libNiFpga.so.17.0.0",
        "lib/libNiFpgaLv.so.17.0.0",
        "lib/libNiRioSrv.so.17.0.0",
        "lib/libRoboRIO_FRC_ChipObject.so.18.0.0",
        "lib/libniriodevenum.so.17.0.0",
        "lib/libniriosession.so.17.0.0",
    ],
    hdrs = glob(["include/**"]),
    includes = [
        "include",
    ],
    linkstatic = True,
    restricted_to = ["@//tools:roborio"],
    visibility = ["//visibility:public"],
)
