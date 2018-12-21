package(default_visibility = ["//visibility:public"])

cc_library(
    name = "type_traits",
    hdrs = [
        "type_traits.h",
    ],
)

cc_test(
    name = "type_traits_test",
    srcs = [
        "type_traits_test.cpp",
    ],
    deps = [
        ":type_traits",
        "//aos/testing:googletest",
    ],
)
