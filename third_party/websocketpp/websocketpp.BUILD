# 3 Clause BSD
licenses(["notice"])

cc_library(
    name = "com_github_zaphoyd_websocketpp",
    hdrs = glob(["websocketpp/**/*.hpp"]),
    defines = ["_WEBSOCKETPP_CPP11_STL_", "ASIO_STANDALONE"],
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = ["@asio"],
)
