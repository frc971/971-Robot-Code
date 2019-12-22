cc_library(
    name = "wpilib",
    linkstatic = True,
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = ["//frc971/wpilib/ahal"],
)

cc_library(
    name = "wpilib_hal",
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = ["//third_party/allwpilib_2019:hal"],
)

cc_library(
    name = "phoenix",
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "@ctre_phoenix_api_cpp_athena//:api-cpp",
        "@ctre_phoenix_api_cpp_headers//:api-cpp",
        "@ctre_phoenix_cci_athena//:cci",
        "@ctre_phoenix_cci_headers//:cci",
        "@ctre_phoenix_diagnostics_athena//:diagnostics",
        "@ctre_phoenix_diagnostics_headers//:diagnostics",
    ],
)
