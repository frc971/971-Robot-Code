load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain")

def _cc_toolchain_make_variables_impl(ctx):
    """Supports make variables for toolchains in a platforms setup.

    The upstream @bazel_tools//tools/cpp:current_cc_toolchain target on its own
    doesn't appear to work with platforms. It only works when using --cpu.
    """
    toolchain = find_cpp_toolchain(ctx)

    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    objcopy = cc_common.get_tool_for_action(
        feature_configuration = feature_configuration,
        action_name = "objcopy_embed_data",
    )

    return [
        platform_common.TemplateVariableInfo({
            "OBJCOPY": objcopy,
        }),
        DefaultInfo(files = toolchain.all_files),
    ]

cc_toolchain_make_variables = rule(
    implementation = _cc_toolchain_make_variables_impl,
    attrs = {
        # This is a dependency of find_cpp_toolchain().
        "_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    fragments = ["cpp"],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
)
