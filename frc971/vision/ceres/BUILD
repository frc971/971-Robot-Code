# Copy of ceres example code from their repo
cc_library(
    name = "pose_graph_3d_lib",
    hdrs = [
        "pose_graph_3d_error_term.h",
        "read_g2o.h",
        "types.h",
    ],
    copts = [
        # Needed to silence GFlags complaints.
        "-Wno-sign-compare",
        "-Wno-unused-parameter",
        "-Wno-format-nonliteral",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/flags:flag",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
