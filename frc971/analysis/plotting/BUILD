cc_library(
    name = "webgl2_plotter",
    srcs = ["webgl2_plotter.cc"],
    hdrs = ["webgl2_plotter.h"],
    linkopts = [
        "-s",
        "USE_WEBGL2=1",
        "-s",
        "FULL_ES3=1",
    ],
    restricted_to = ["//tools:web"],
    visibility = ["//visibility:public"],
    deps = ["@org_tuxfamily_eigen//:eigen"],
)

cc_library(
    name = "webgl2_animator",
    srcs = ["webgl2_animator.cc"],
    hdrs = ["webgl2_animator.h"],
    linkopts = [
        "-s",
        "USE_WEBGL2=1",
    ],
    restricted_to = ["//tools:web"],
    visibility = ["//visibility:public"],
    deps = [
        ":webgl2_plotter",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
