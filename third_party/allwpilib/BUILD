licenses(["notice"])

# Names of WPILib "devices" I don't want to deal with making trivial updates to
# chop out various ugliness or have to vet for sanity.
_excluded_devices = [
    "AnalogGyro",
    "SerialPort",
    "SerialHelper",
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

py_binary(
    name = "generate_FRCUsageReporting",
    srcs = [
        "generate_FRCUsageReporting.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

genrule(
    name = "do_generate_FRCUsageReporting",
    srcs = [
        "hal/src/generate/FRCUsageReporting.h.in",
        "hal/src/generate/Instances.txt",
        "hal/src/generate/ResourceType.txt",
    ],
    outs = [
        "hal/src/main/native/include/hal/FRCUsageReporting.h",
    ],
    cmd = " ".join([
        "$(location :generate_FRCUsageReporting)",
        "$(location hal/src/generate/FRCUsageReporting.h.in)",
        "$(location hal/src/generate/Instances.txt)",
        "$(location hal/src/generate/ResourceType.txt)",
        "$(location hal/src/main/native/include/hal/FRCUsageReporting.h)",
    ]),
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        ":generate_FRCUsageReporting",
    ],
)

cc_library(
    name = "hal",
    srcs = glob(
        include = [
            "hal/src/main/native/athena/*.cpp",
            "hal/src/main/native/cpp/cpp/*.cpp",
            "hal/src/main/native/cpp/*.cpp",
            "hal/src/main/native/athena/ctre/*.cpp",
            "hal/src/main/native/shared/handles/*.cpp",
            "hal/src/main/native/cpp/handles/*.cpp",
        ],
        exclude = ["**/%s.*" % d for d in _excluded_devices],
    ),
    hdrs = _hal_h_hdrs + _hal_hpp_hdrs + [
        "hal/src/main/native/include/hal/FRCUsageReporting.h",
    ],
    copts = [
        "-Wno-unused-parameter",
        "-Wno-cast-align",
    ],
    includes = _hal_header_dirs,
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//third_party:__pkg__"],
    deps = [
        "//aos/logging",
        "//third_party/allwpilib/wpiutil",
        "@allwpilib_ni_libraries//:ni-libraries",
    ],
)
