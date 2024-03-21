load("@npm//:defs.bzl", "npm_link_all_packages")
load("//tools/build_rules:js.bzl", "ng_pkg")

npm_link_all_packages(name = "node_modules")

ng_pkg(
    name = "match_list",
    extra_srcs = [
        "//scouting/www:app_common_css",
    ],
    deps = [
        ":node_modules",
        "//:node_modules/flatbuffers",
    ],
)
