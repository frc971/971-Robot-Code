cc_library(
    name = "ni-libraries",
    srcs = [
        "src/lib/chipobject/libNiFpga.so.18.0.0",
        "src/lib/chipobject/libNiFpgaLv.so.18.0.0",
        "src/lib/chipobject/libNiRioSrv.so.18.0.0",
        "src/lib/chipobject/libRoboRIO_FRC_ChipObject.so.19.0.0",
        "src/lib/chipobject/libniriodevenum.so.18.0.0",
        "src/lib/chipobject/libniriosession.so.18.0.0",
        "src/lib/netcomm/libFRC_NetworkCommunication.so.19.0.0",
    ],
    hdrs = glob(["src/include/**"]),
    includes = [
        "src/include",
    ],
    linkstatic = True,
    restricted_to = ["@//tools:roborio"],
    visibility = ["//visibility:public"],
)
