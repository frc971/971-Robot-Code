load("//tools/build_rules:js.bzl", "rollup_bundle", "ts_project")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "starter_files",
    srcs = [
        "starter.html",
        "starter_main_bundle.min.js",
        "styles.css",
    ],
)

ts_project(
    name = "starter_main",
    srcs = [
        "starter_handler.ts",
        "starter_main.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network:connect_ts_fbs",
        "//aos/network:message_bridge_client_ts_fbs",
        "//aos/network/www:proxy",
        "//aos/starter:starter_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

rollup_bundle(
    name = "starter_main_bundle",
    entry_point = "starter_main.ts",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":starter_main",
    ],
)
