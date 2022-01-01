load("@bazel_gazelle//:def.bzl", "gazelle")

exports_files([
    "tsconfig.json",
    "rollup.config.js",
])

# gazelle:prefix github.com/frc971/971-Robot-Code
# gazelle:build_file_name BUILD
# gazelle:proto disable
# gazelle:go_generate_proto false
# gazelle:exclude third_party
# gazelle:exclude external

gazelle(
    name = "gazelle",
    visibility = ["//tools/lint:__subpackages__"],
)
