licenses(["notice"])

genrule(
    name = "sctp_copy",
    srcs = [
        "src/include/netinet/sctp.h.in",
    ],
    outs = [
        "src/include/netinet/sctp.h",
    ],
    cmd = "cp $< $@",
)

cc_library(
    name = "sctp",
    srcs = [
        "src/lib/addrs.c",
        "src/lib/bindx.c",
        "src/lib/opt_info.c",
        "src/lib/peeloff.c",
        "src/lib/recvmsg.c",
        "src/lib/sendmsg.c",
    ],
    hdrs = [
        "src/include/netinet/sctp.h",
    ],
    copts = [
        "-Wno-cast-align",
        "-Wno-cast-qual",
    ],
    includes = [
        "src/include",
    ],
    visibility = ["//visibility:public"],
)
