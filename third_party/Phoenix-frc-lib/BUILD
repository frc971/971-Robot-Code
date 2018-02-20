# It also restricts use to only with CTRE products and a roboRIO in FRC.
licenses(["permissive"])

# Excludes motor/sensor code because those have reliances on WPILib.
cc_library(
    name = "phoenix",
    srcs = glob(
        include = [
            "cpp/src/**/*.cpp",
            "libraries/driver/include/src/**/*.cpp",
        ],
        exclude = [
            "**/Tasking/**",
            "cpp/src/RCRadio3Ch.cpp",
            "cpp/src/CompileTest.cpp",
            "**/MotorControl/**",
            "**/Sensors/**",
        ],
    ) + [
        "libraries/driver/lib/libCTRE_PhoenixCCI.a",
    ],
    hdrs = glob(
        include = [
            "cpp/include/**/*.h",
            "libraries/driver/include/**/*.h",
        ],
        exclude = [
            "**/Tasking/**",
            "**/MotorControl/**",
            "**/Sensors/**",
            "cpp/include/ctre/phoenix/RCRadio3Ch.h",
        ],
    ),
    includes = [
        "cpp/include",
        "libraries/driver/include",
    ],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:wpilib",
    ],
)
