load("@org_frc971//tools/build_rules:select.bzl", "compiler_select")

cc_library(
    name = "s3",
    srcs = glob(["aws-cpp-sdk-s3/source/**/*.cpp"]),
    hdrs = glob(["aws-cpp-sdk-s3/include/**/*.h"]),
    includes = ["aws-cpp-sdk-s3/include"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aws-c-auth",
        ":core",
    ],
)

genrule(
    name = "gen_SDKConfig",
    outs = ["aws-cpp-sdk-core/include/aws/core/SDKConfig.h"],
    cmd = "echo '#undef USE_AWS_MEMORY_MANAGEMENT' > $@",
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "core",
    srcs = glob(
        include = ["aws-cpp-sdk-core/source/**/*.cpp"],
        exclude = [
            "aws-cpp-sdk-core/source/utils/crypto/*/*.cpp",
            "aws-cpp-sdk-core/source/platform/**/*.cpp",
            "aws-cpp-sdk-core/source/platform/windows/**/*.cpp",
            # net/*.cpp is for not-(linux or windows), so exclude everything in there.
            "aws-cpp-sdk-core/source/net/**/*.cpp",
            "aws-cpp-sdk-core/source/http/windows/**/*.cpp",
        ],
    ) + glob([
        "aws-cpp-sdk-core/source/utils/crypto/openssl/*.cpp",
        "aws-cpp-sdk-core/source/utils/crypto/factory/*.cpp",
        "aws-cpp-sdk-core/source/platform/linux-shared/**/*.cpp",
        "aws-cpp-sdk-core/source/net/linux-shared/*.cpp",
    ]) + [
        ":gen_SDKConfig",
    ],
    hdrs = glob(
        include = ["aws-cpp-sdk-core/include/**/*.h"],
        exclude = [
            "aws-cpp-sdk-core/include/aws/core/utils/crypto/*/*.h",
            "aws-cpp-sdk-core/include/aws/core/http/windows/**/*.h",
        ],
    ) + glob([
        "aws-cpp-sdk-core/include/aws/core/utils/crypto/openssl/*.h",
    ]),
    copts = [
        "-DAWS_SDK_VERSION_MAJOR=10",
        "-DAWS_SDK_VERSION_MINOR=34",
        "-DAWS_SDK_VERSION_PATCH=\"\\\"BRT\"\\\"",
        "-DENABLE_OPENSSL_ENCRYPTION",
        "-DENABLE_CURL_CLIENT",
        "-Wno-cast-align",
        "-Wno-cast-qual",
        "-Wno-format-nonliteral",
    ],
    includes = ["aws-cpp-sdk-core/include"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aws-c-auth",
        ":aws-c-common",
        ":aws-c-http",
        ":crt",
        "@boringssl//:crypto",
        "@com_github_curl_curl//:curl",
    ],
)

genrule(
    name = "gen_Config",
    outs = ["crt/aws-crt-cpp/include/aws/crt/Config.h"],
    cmd = "echo '#define AWS_CRT_CPP_VERSION \"1.10.34\"' > $@",
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "crt",
    srcs = glob(["crt/aws-crt-cpp/source/**/*.cpp"]),
    hdrs = glob(["crt/aws-crt-cpp/include/**/*.h"]) + [
        ":gen_Config",
    ],
    copts = [
        "-Wno-sign-compare",
        "-Wno-cast-qual",
        "-Wno-tautological-type-limit-compare",
        "-Wno-missing-field-initializers",
    ],
    includes = ["crt/aws-crt-cpp/include"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aws-c-auth",
        ":aws-c-common",
        ":aws-c-event-stream",
        ":aws-c-mqtt",
        ":aws-c-s3",
        ":aws-c-sdkutils",
    ],
)

genrule(
    name = "gen_config",
    outs = ["crt/aws-crt-cpp/crt/aws-c-common/include/aws/common/config.h"],
    cmd = "\n".join([
        "cat >$@ <<END",
        "#define AWS_HAVE_GCC_OVERFLOW_MATH_EXTENSIONS 1",
        "#define AWS_HAVE_GCC_INLINE_ASM 1",
        "#undef AWS_HAVE_MSVC_MULX",
        "#define AWS_HAVE_EXECINFO 1",
        "#define AWS_AFFINITY_METHOD 0",
        "END",
    ]),
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "aws-c-common",
    srcs = glob([
        "crt/aws-crt-cpp/crt/aws-c-common/source/*.c",
        "crt/aws-crt-cpp/crt/aws-c-common/source/external/*.c",
        "crt/aws-crt-cpp/crt/aws-c-common/source/posix/*.c",
    ]) + [
        ":gen_config",
    ] + select({
        # See the paths in crt/aws-crt-cpp/crt/aws-c-common/CMakeLists.txt for the appropriate globs for each architecture.
        "@//tools:cpu_k8": glob(
            include = [
                "crt/aws-crt-cpp/crt/aws-c-common/source/arch/intel/*.c",
                "crt/aws-crt-cpp/crt/aws-c-common/source/arch/intel/asm/*.c",
            ],
            exclude = [
                # We don't build with AVX, see crt/aws-crt-cpp/crt/aws-c-common/CMakeLists.txt for details of the macros that need to be set if this is enabled.
                "crt/aws-crt-cpp/crt/aws-c-common/source/arch/intel/encoding_avx2.c",
            ],
        ),
        "@//tools:cpu_arm64": glob([
            "crt/aws-crt-cpp/crt/aws-c-common/source/arch/arm/asm/*.c",
        ]),
        "@//tools:cpu_armv7": glob([
            "crt/aws-crt-cpp/crt/aws-c-common/source/arch/arm/asm/*.c",
        ]),
        "//conditions:default": [],
    }),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-common/include/**/*.h"]),
    copts = [
        "-Wno-cast-align",
        "-Wno-cast-qual",
        "-Wno-sign-compare",
        "-Wno-format-nonliteral",
    ] + compiler_select({
        "clang": [],
        "gcc": [
            "-Wno-old-style-declaration",
        ],
    }),
    includes = ["crt/aws-crt-cpp/crt/aws-c-common/include"],
    target_compatible_with = ["@platforms//os:linux"],
    textual_hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-common/include/**/*.inl"]),
    visibility = ["//visibility:public"],
)

# -march=armv8-a+crc
cc_library(
    name = "aws-c-event-stream",
    srcs = glob(["crt/aws-crt-cpp/crt/aws-c-event-stream/source/*.c"]) + select({
        "@//tools:cpu_k8": glob(["crt/aws-crt-cpp/crt/aws-c-event-stream/source/intel/asm/*.c"]),
        "@//tools:cpu_arm64": glob(["crt/aws-crt-cpp/crt/aws-c-event-stream/source/arm/*.c"]),
        "@//tools:cpu_armv7": glob(["crt/aws-crt-cpp/crt/aws-c-event-stream/source/arm/*.c"]),
        "//conditions:default": [],
    }),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-event-stream/include/**/*.h"]),
    copts = [
        "-Wno-cast-align",
        "-Wno-cast-qual",
    ],
    includes = ["crt/aws-crt-cpp/crt/aws-c-event-stream/include"],
    deps = [
        ":aws-c-common",
        ":aws-c-io",
        ":aws-checksums",
    ],
)

cc_library(
    name = "aws-checksums",
    srcs = glob(["crt/aws-crt-cpp/crt/aws-checksums/source/*.c"]) + select({
        "@//tools:cpu_k8": glob(["crt/aws-crt-cpp/crt/aws-checksums/source/intel/asm/*.c"]),
        "@//tools:cpu_arm64": glob(["crt/aws-crt-cpp/crt/aws-checksums/source/arm/*.c"]),
        "@//tools:cpu_armv7": glob(["crt/aws-crt-cpp/crt/aws-checksums/source/arm/*.c"]),
        "//conditions:default": [],
    }),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-checksums/include/**/*.h"]),
    copts = [
        "-Wno-cast-qual",
        "-Wno-cast-align",
        "-Wno-implicit-function-declaration",
    ],
    includes = ["crt/aws-crt-cpp/crt/aws-checksums/include"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aws-c-common",
    ],
)

cc_library(
    name = "aws-c-cal",
    srcs = glob([
        "crt/aws-crt-cpp/crt/aws-c-cal/source/*.c",
        "crt/aws-crt-cpp/crt/aws-c-cal/source/unix/*.c",
    ]),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-cal/include/**/*.h"]),
    copts = [
        "-DOPENSSL_IS_AWSLC",
        "-Wno-incompatible-pointer-types",
        "-Wno-unused-function",
        "-Wno-unused-parameter",
        "-Wno-cast-align",
        "-Wno-cast-qual",
    ],
    includes = ["crt/aws-crt-cpp/crt/aws-c-cal/include"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aws-c-common",
        "@boringssl//:crypto",
    ],
)

cc_library(
    name = "aws-c-s3",
    srcs = glob(["crt/aws-crt-cpp/crt/aws-c-s3/source/**/*.c"]),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-s3/include/**/*.h"]),
    copts = [
        "-Wno-cast-align",
        "-Wno-cast-qual",
    ],
    includes = ["crt/aws-crt-cpp/crt/aws-c-s3/include"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aws-c-auth",
        ":aws-c-common",
        ":aws-checksums",
    ],
)

cc_library(
    name = "aws-c-compression",
    srcs = glob(["crt/aws-crt-cpp/crt/aws-c-compression/source/*.c"]),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-compression/include/**/*.h"]),
    copts = ["-Wno-cast-qual"],
    includes = ["crt/aws-crt-cpp/crt/aws-c-compression/include"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aws-c-common",
    ],
)

cc_library(
    name = "aws-c-http",
    srcs = glob(["crt/aws-crt-cpp/crt/aws-c-http/source/**/*.c"]),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-http/include/**/*.h"]),
    copts = [
        "-Wno-unused-but-set-variable",
        "-Wno-cast-align",
        "-Wno-cast-qual",
    ],
    includes = ["crt/aws-crt-cpp/crt/aws-c-http/include"],
    target_compatible_with = ["@platforms//os:linux"],
    textual_hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-http/include/**/*.def"]),
    deps = [
        ":aws-c-common",
        ":aws-c-compression",
        ":aws-c-io",
    ],
)

cc_library(
    name = "aws-c-sdkutils",
    srcs = glob(["crt/aws-crt-cpp/crt/aws-c-sdkutils/source/**/*.c"]),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-sdkutils/include/**/*.h"]),
    copts = [
        "-Wno-cast-align",
        "-Wno-cast-qual",
    ],
    includes = ["crt/aws-crt-cpp/crt/aws-c-sdkutils/include"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aws-c-common",
    ],
)

cc_library(
    name = "aws-c-auth",
    srcs = glob(["crt/aws-crt-cpp/crt/aws-c-auth/source/**/*.c"]),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-auth/include/**/*.h"]),
    copts = [
        "-Wno-cast-align",
        "-Wno-cast-qual",
    ],
    includes = ["crt/aws-crt-cpp/crt/aws-c-auth/include"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aws-c-common",
        ":aws-c-http",
        ":aws-c-io",
        ":aws-c-sdkutils",
    ],
)

cc_library(
    name = "aws-c-mqtt",
    srcs = glob(["crt/aws-crt-cpp/crt/aws-c-mqtt/source/**/*.c"]),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-mqtt/include/**/*.h"]),
    copts = [
        "-Wno-cast-qual",
        "-Wno-cast-align",
        "-DAWS_MQTT_WITH_WEBSOCKETS",
    ],
    includes = ["crt/aws-crt-cpp/crt/aws-c-mqtt/include"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aws-c-common",
        ":aws-c-http",
        ":aws-c-io",
    ],
)

cc_library(
    name = "aws-c-io",
    srcs = glob([
        "crt/aws-crt-cpp/crt/aws-c-io/source/*.c",
        "crt/aws-crt-cpp/crt/aws-c-io/source/linux/*.c",
        "crt/aws-crt-cpp/crt/aws-c-io/source/s2n/*.c",
        "crt/aws-crt-cpp/crt/aws-c-io/source/posix/*.c",
    ]),
    hdrs = glob(["crt/aws-crt-cpp/crt/aws-c-io/include/**/*.h"]) + [
        "crt/aws-crt-cpp/crt/aws-c-io/source/pkcs11_private.h",
    ] + glob([
        "crt/aws-crt-cpp/crt/aws-c-io/source/pkcs11/v2.40/*.h",
    ]),
    copts = [
        "-DUSE_S2N",
        "-DAWS_USE_EPOLL",
        "-Wno-cast-align",
        "-Wno-cast-qual",
        "-Wno-sign-compare",
        "-Wno-unused-parameter",
    ],
    includes = ["crt/aws-crt-cpp/crt/aws-c-io/include"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aws-c-cal",
        ":aws-c-common",
        ":s2n",
    ],
)

cc_library(
    name = "s2n",
    srcs = glob([
        "crt/aws-crt-cpp/crt/s2n/**/*.h",
        "crt/aws-crt-cpp/crt/s2n/tls/**/*.c",
        "crt/aws-crt-cpp/crt/s2n/error/**/*.c",
        "crt/aws-crt-cpp/crt/s2n/utils/**/*.c",
        "crt/aws-crt-cpp/crt/s2n/stuffer/**/*.c",
        "crt/aws-crt-cpp/crt/s2n/crypto/**/*.c",
        "crt/aws-crt-cpp/crt/s2n/pq-crypto/*.c",
    ]),
    hdrs = ["crt/aws-crt-cpp/crt/s2n/api/s2n.h"],
    copts = [
        "-Iexternal/aws_sdk/crt/aws-crt-cpp/crt/s2n",
        "-DS2N_NO_PQ",
        "-Wno-unknown-pragmas",
        "-Wno-cast-align",
        "-Wno-cast-qual",
        "-Wno-unused-parameter",
        "-Wno-sign-compare",
    ],
    includes = ["crt/aws-crt-cpp/crt/s2n/api"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@boringssl//:crypto",
    ],
)
