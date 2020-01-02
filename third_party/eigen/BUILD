licenses(["notice"])

load("@//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "eigen",
    srcs = glob(["Eigen/src/**/*.h"]),
    hdrs = glob(
        ["Eigen/*"],
        exclude = [
            # Stuff that we don't have the dependencies for.
            "Eigen/CholmodSupport",
            "Eigen/MetisSupport",
            "Eigen/PaStiXSupport",
            "Eigen/PardisoSupport",
            "Eigen/SPQRSupport",
            "Eigen/SuperLUSupport",
            "Eigen/UmfPackSupport",
        ],
    ) + ["unsupported/Eigen/MatrixFunctions"] + glob([
        "unsupported/Eigen/src/MatrixFunctions/*.h",
    ]),
    compatible_with = mcu_cpus + ["@//tools:web"],
    includes = ["."],
    visibility = ["//visibility:public"],
)
