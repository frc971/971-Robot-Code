licenses(["notice"])

cc_library(
    name = "wpimath",
    hdrs = glob(
        [
            "src/main/native/include/frc/fmt/**",
            "src/main/native/include/units/**",
        ],
    ),
    includes = [
        "src/main/native/include",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
)
