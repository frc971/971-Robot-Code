cc_library(
    name = "ni-libraries",
    srcs = [
        "src/lib/chipobject/libRoboRIO_FRC_ChipObject.so.20.0.0",
        "src/lib/netcomm/libFRC_NetworkCommunication.so.20.0.0",
        "src/lib/runtime/libNiFpga.so.19.0.0",
        "src/lib/runtime/libNiFpgaLv.so.19.0.0",
        "src/lib/runtime/libNiRioSrv.so.19.0.0",
        "src/lib/runtime/libni_emb.so.12.0.0",
        "src/lib/runtime/libni_rtlog.so.2.8.0",
        "src/lib/runtime/libnirio_emb_can.so.16.0.0",
        "src/lib/runtime/libniriodevenum.so.19.0.0",
        "src/lib/runtime/libniriosession.so.18.0.0",
    ],
    hdrs = glob(["src/include/**"]),
    includes = [
        "src/include",
    ],
    linkopts = ["-ldl"],
    linkstatic = True,
    restricted_to = ["@//tools:roborio"],
    visibility = ["//visibility:public"],
)
