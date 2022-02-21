licenses(["notice"])

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
        "unsupported/Eigen/CXX11/Tensor",
        "unsupported/Eigen/SpecialFunctions",
        "unsupported/Eigen/src/SpecialFunctions/**/*.h",
        "unsupported/Eigen/CXX11/src/util/*.h",
        "unsupported/Eigen/CXX11/src/Tensor/*.h",
    ]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
