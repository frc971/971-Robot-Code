cc_library(
    name = "akaze",
    srcs = [
        "akaze.cpp",
    ],
    hdrs = [
        "akaze.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":akaze_features",
        "//third_party:opencv",
    ],
)

cc_library(
    name = "akaze_config",
    hdrs = [
        "AKAZEConfig.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:opencv",
    ],
)

cc_library(
    name = "akaze_features",
    srcs = [
        "AKAZEFeatures.cpp",
    ],
    hdrs = [
        "AKAZEFeatures.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":akaze_config",
        ":fed",
        ":nldiffusion_functions",
        ":t_evolution",
        ":utils",
        "//third_party:opencv",
    ],
)

cc_library(
    name = "fed",
    srcs = [
        "fed.cpp",
    ],
    hdrs = [
        "fed.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:opencv",
    ],
)

cc_library(
    name = "nldiffusion_functions",
    srcs = [
        "nldiffusion_functions.cpp",
    ],
    hdrs = [
        "nldiffusion_functions.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:opencv",
    ],
)

cc_library(
    name = "t_evolution",
    hdrs = [
        "TEvolution.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:opencv",
    ],
)

cc_library(
    name = "utils",
    hdrs = [
        "utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:opencv",
    ],
)
