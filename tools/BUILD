load("@build_bazel_rules_nodejs//:index.bzl", "nodejs_binary")

package(default_visibility = ["//visibility:public"])

exports_files(["test_sharding_compliant"])

# Don't use these directly! Use //tools/build_rules/*.bzl instead.
config_setting(
    name = "compiler_clang",
    flag_values = {
        "@bazel_tools//tools/cpp:compiler": "clang",
    },
)

config_setting(
    name = "compiler_gcc",
    flag_values = {
        "@bazel_tools//tools/cpp:compiler": "gcc",
    },
)

# TODO(phil): Get rid of these and just use the constraint values directly
# everywhere.
config_setting(
    name = "cpu_k8",
    constraint_values = ["@platforms//cpu:x86_64"],
)

config_setting(
    name = "cpu_roborio",
    constraint_values = ["@//tools/platforms/hardware:roborio"],
)

config_setting(
    name = "cpu_cortex_m4f",
    constraint_values = ["@//tools/platforms/hardware:cortex_m4f"],
)

config_setting(
    name = "cpu_cortex_m0plus",
    constraint_values = ["@//tools/platforms/hardware:cortex_m0plus"],
)

# TODO(phil): Re-enable this with proper support.
#config_setting(
#    name = "cpu_cortex_m4f_k22",
#    constraint_values = ["@platforms//cpu:?"],
#)

config_setting(
    name = "cpu_armhf",
    constraint_values = [
        "@platforms//cpu:armv7",
        "//tools/platforms/hardware:raspberry_pi",
    ],
)

config_setting(
    name = "cpu_armv7",
    constraint_values = [
        "@platforms//cpu:armv7",
    ],
)

config_setting(
    name = "cpu_arm64",
    constraint_values = ["@platforms//cpu:arm64"],
)

config_setting(
    name = "has_asan",
    values = {"define": "have_asan=true"},
)

config_setting(
    name = "has_tsan",
    values = {"define": "have_tsan=true"},
)

config_setting(
    name = "has_ubsan",
    values = {"define": "have_ubsan=true"},
)

nodejs_binary(
    name = "tsc_wrapped_with_angular",
    data = [
        "@npm//@angular/compiler-cli",
        "@npm//@bazel/typescript",
    ],
    entry_point = "@npm//:node_modules/@bazel/typescript/internal/tsc_wrapped/tsc_wrapped.js",
    target_compatible_with = ["@platforms//cpu:x86_64"],
)
