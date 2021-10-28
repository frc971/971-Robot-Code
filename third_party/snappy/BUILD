genrule(
    name = "make_stubs_public",
    outs = ["snappy-stubs-public.h"],
    cmd = "echo \"#include <cstddef>\n#include <sys/uio.h>\n\" > $@",
)

cc_library(
    name = "snappy",
    srcs = [
        "snappy.cc",
        "snappy-internal.h",
        "snappy-sinksource.cc",
        "snappy-sinksource.h",
        "snappy-stubs-internal.cc",
        "snappy-stubs-internal.h",
    ],
    hdrs = [
        "snappy.h",
        "snappy-stubs-public.h",
    ],
    copts = ["-Wno-sign-compare"],
)
