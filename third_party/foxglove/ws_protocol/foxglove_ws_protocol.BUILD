# MIT License
licenses(["notice"])

cc_library(
    name = "com_github_foxglove_ws-protocol",
    hdrs = ["cpp/foxglove-websocket/include/foxglove/websocket/server.hpp"],
    includes = ["cpp/foxglove-websocket/include/"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_nlohmann_json//:json",
        "@com_github_zaphoyd_websocketpp",
    ],
)

cc_binary(
    name = "example_server",
    srcs = ["cpp/examples/example_server.cpp"],
    deps = [":com_github_foxglove_ws-protocol"],
)
