load("@ci_configure//:ci.bzl", "RUNNING_IN_CI")

sh_binary(
    name = "gofmt",
    srcs = ["gofmt.sh"],
    data = [
        "@go_sdk//:bin/gofmt",
    ],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "buildifier",
    srcs = ["buildifier.sh"],
    data = [
        "@com_github_bazelbuild_buildtools//buildifier",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "prettier",
    srcs = ["prettier.sh"],
    data = [
        "@npm//prettier/bin:prettier",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "run-ci",
    srcs = [
        "run-ci.sh",
    ],
    data = [
        ":buildifier",
        ":gofmt",
        ":prettier",
        "//:gazelle-runner",
        "//tools/go:mirror_go_repos",
        "//tools/go:tweak_gazelle_go_deps",
        "@go_sdk//:bin/go",
    ],
    env = {
        # Prevent CI errors like:
        #   failed to initialize build cache at
        #   /var/lib/buildkite-agent/.cache/go-build: permission denied
        "RUNNING_IN_CI": "1" if RUNNING_IN_CI else "0",
    },
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)
