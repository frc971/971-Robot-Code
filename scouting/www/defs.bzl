load("@aspect_bazel_lib//lib:copy_to_directory.bzl", "copy_to_directory_bin_action")

def _assemble_static_files_impl(ctx):
    out_dir = ctx.actions.declare_directory(ctx.label.name)

    copy_to_directory_bin = ctx.toolchains["@aspect_bazel_lib//lib:copy_to_directory_toolchain_type"].copy_to_directory_info.bin

    copy_to_directory_bin_action(
        ctx,
        dst = out_dir,
        name = ctx.label.name,
        copy_to_directory_bin = copy_to_directory_bin,
        files = ctx.files.srcs + ctx.attr.app_files.files.to_list(),
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
        "srcs": attr.label_list(
            mandatory = True,
            allow_files = True,
        ),
        "replace_prefixes": attr.string_dict(
            mandatory = True,
        ),
    },
    toolchains = ["@aspect_bazel_lib//lib:copy_to_directory_toolchain_type"],
)

def _assemble_service_worker_files_impl(ctx):
    args = ctx.actions.args()
    args.add_all(ctx.attr._package.files, before_each = "--input_dir", expand_directories = False)
    args.add_all(ctx.outputs.outs, before_each = "--output")
    args.add_all(ctx.attr.outs_as_strings, before_each = "--relative_output")
    ctx.actions.run(
        inputs = ctx.attr._package.files,
        outputs = ctx.outputs.outs,
        executable = ctx.executable._tool,
        arguments = [args],
        mnemonic = "AssembleAngularServiceWorker",
    )

_assemble_service_worker_files = rule(
    implementation = _assemble_service_worker_files_impl,
    attrs = {
        "outs": attr.output_list(
            allow_empty = False,
            mandatory = True,
        ),
        "outs_as_strings": attr.string_list(
            allow_empty = False,
            mandatory = True,
        ),
        "_package": attr.label(
            default = "//:node_modules/@angular/service-worker",
        ),
        "_tool": attr.label(
            default = "//tools/build_rules/js:assemble_service_worker_files",
            cfg = "exec",
            executable = True,
        ),
    },
)

def assemble_service_worker_files(outs, **kwargs):
    _assemble_service_worker_files(
        outs = outs,
        outs_as_strings = outs,
        **kwargs
    )
