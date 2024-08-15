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
