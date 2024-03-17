load("@npm//:defs.bzl", "npm_link_all_packages")
load("//tools/build_rules:js.bzl", "ng_pkg")

npm_link_all_packages(name = "node_modules")

ng_pkg(
    name = "rpc",
    extra_srcs = [
        "public-api.ts",
    ],
    generate_public_api = False,
    deps = [
        ":node_modules",
        "//:node_modules/flatbuffers",
    ],
)
