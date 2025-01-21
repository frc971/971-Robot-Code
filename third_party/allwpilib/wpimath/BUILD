licenses(["notice"])

cc_library(
    name = "wpimath",
    hdrs = glob(
        [
            "src/main/native/include/frc/fmt/**",
            "src/main/native/include/units/**",
            "src/main/native/thirdparty/gcem/include/**"
        ],
    ),
    includes = [
        "src/main/native/include",
        "src/main/native/thirdparty/gcem/include"
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
)
