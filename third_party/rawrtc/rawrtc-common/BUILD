cc_library(
    name = "rawrtcc",
    srcs = glob(["src/**/*.c"]),
    hdrs = glob(["include/**/*.h"]),
    defines = [
        "RAWRTC_DEBUG_LEVEL=0",
    ],
    includes = ["include/"],
    visibility = ["//visibility:public"],
    deps = ["@com_github_rawrtc_rew//:rew"],
)
