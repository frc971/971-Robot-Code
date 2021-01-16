# usrsctp is only actually being used for the CRC function, and getting
# the entire library building was being obnoxious.
cc_library(
    name = "usrsctp_crc32",
    srcs = ["usrsctplib/netinet/sctp_crc32.c"],
    hdrs = glob(["usrsctplib/**/*.h"]),
    copts = [
        "-Wno-cast-qual",
        "-Wno-cast-align",
        "-Wno-unused-parameter",
        "-Wno-incompatible-pointer-types-discards-qualifiers",
    ],
    defines = [
        "__Userspace_os_Linux",
        "__Userspace__",
        "SCTP_SIMPLE_ALLOCATOR",
    ],
    includes = ["usrsctplib/"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
