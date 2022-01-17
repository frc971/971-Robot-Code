licenses(["notice"])

cc_library(
    name = "CImg",
    hdrs = glob([
        "CImg.h",
        "plugins/*.h",
    ]),
    target_compatible_with = ["@platforms//os:linux"],
    # This library has undefined behavior, so don't use it anywhere else without
    # evaluating carefully. Note that this is different from most compiler
    # warnings about undefined behavior: in several places it is statically
    # always undefined behavior, not just code that looks like it might have
    # undefined behavior when executed sometimes.
    #
    # Also, if the compiler finds several, there are probably more subtle ones
    # that the compiler doesn't notice.
    visibility = ["//y2019/image_streamer:__pkg__"],
    deps = [
        "//third_party/libjpeg",
    ],
)
