licenses(["notice"])

# Names of WPILib "devices" I don't want to deal with making trivial updates to
# chop out various ugliness or have to vet for sanity.
_excluded_devices = [
    "AnalogGyro",
    "SerialPort",
    "SerialHelper",
    "OSSerialPort",
    "visa",
]

# Header files we don't want to have.
_bad_hdrs = ([
    "hal/include/HAL/LabVIEW/HAL.h",
] + ["**/%s.*" % d for d in _excluded_devices])

_hal_header_dirs = [
    "hal/src/main/native/athena",
    "hal/src/main/native/include",
]

_hal_h_hdrs = glob(
    [d + "/**/*.h" for d in _hal_header_dirs],
    exclude = _bad_hdrs,
)

_hal_hpp_hdrs = glob(
    [d + "/**/*.hpp" for d in _hal_header_dirs],
    exclude = _bad_hdrs,
)

cc_library(
    name = "hal",
    srcs = glob(
        [
            "hal/src/main/native/athena/*.cpp",
            "hal/src/main/native/athena/cpp/*.cpp",
            "hal/src/main/native/athena/ctre/*.cpp",
            "hal/src/main/native/shared/handles/*.cpp",
        ],
        exclude = ["**/%s.*" % d for d in _excluded_devices],
    ),
    hdrs = _hal_h_hdrs + _hal_hpp_hdrs,
    copts = [
        "-Wno-unused-parameter",
        "-Wno-switch-enum",
        "-Wno-attributes",
        "-Wno-cast-align",
        "-Wno-cast-qual",
        "-Wno-deprecated-declarations",
        "-Wno-error",
        "-Wno-unused-const-variable",
    ],
    defines = ["WPILIB2018=1"],
    includes = _hal_header_dirs,
    linkopts = ["-lpthread"],
    restricted_to = ["//tools:roborio"],
    visibility = [
        "//third_party:__pkg__",
        "//third_party/Phoenix-frc-lib:__pkg__",
    ],
    deps = [
        "//aos/logging",
        "//third_party/wpiutil_2018:wpiutil",
        "@allwpilib_ni_libraries_repo_2018//:ni-libraries",
    ],
)
