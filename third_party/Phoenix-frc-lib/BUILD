# It also restricts use to only with CTRE products and a roboRIO in FRC.
licenses(["permissive"])

cc_library(
    name = "libs",
    srcs = [
        "libraries/driver/lib/libCTRE_PhoenixCCI.a",
    ],
    restricted_to = ["//tools:roborio"],
)

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
    ),
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
        ":libs",
        "//third_party/allwpilib_2018:hal",
    ],
)
