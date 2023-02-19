load("@aspect_bazel_lib//lib:copy_to_directory.bzl", "copy_to_directory_bin_action")

def _assemble_static_files_impl(ctx):
    out_dir = ctx.actions.declare_directory(ctx.label.name)

    copy_to_directory_bin = ctx.toolchains["@aspect_bazel_lib//lib:copy_to_directory_toolchain_type"].copy_to_directory_info.bin

    copy_to_directory_bin_action(
        ctx,
        dst = out_dir,
        name = ctx.label.name,
        copy_to_directory_bin = copy_to_directory_bin,
        files = ctx.files.pictures + ctx.attr.app_files.files.to_list(),
        replace_prefixes = ctx.attr.replace_prefixes,
    )

    return [DefaultInfo(
        files = depset([out_dir]),
        runfiles = ctx.runfiles([out_dir]),
    )]

assemble_static_files = rule(
    implementation = _assemble_static_files_impl,
    attrs = {
        "app_files": attr.label(
            mandatory = True,
        ),
        "pictures": attr.label_list(
            mandatory = True,
        ),
        "replace_prefixes": attr.string_dict(
            mandatory = True,
        ),
    },
    toolchains = ["@aspect_bazel_lib//lib:copy_to_directory_toolchain_type"],
)
