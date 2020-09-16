def _jinja2_template_impl(ctx):
    out = ctx.actions.declare_file(ctx.attr.name)

    ctx.actions.run_shell(
        inputs = ctx.files.src,
        tools = [ctx.executable._jinja2],
        progress_message = "Generating " + out.short_path,
        outputs = [out],
        command = ctx.executable._jinja2.path + " " + ctx.files.src[0].path + " '" + str(ctx.attr.parameters) + "' " + out.path,
    )

    return [DefaultInfo(files = depset([out])), OutputGroupInfo(out = depset([out]))]

jinja2_template = rule(
    attrs = {
        "src": attr.label(
            mandatory = True,
            allow_single_file = True,
            doc = """The jinja2 template file to expand.""",
        ),
        "parameters": attr.string_dict(
            mandatory = True,
            doc = """The parameters to supply to Jinja2.""",
        ),
        "_jinja2": attr.label(
            default = "//tools/build_rules:jinja2_generator",
            cfg = "host",
            executable = True,
        ),
    },
    implementation = _jinja2_template_impl,
    doc = """Expands a jinja2 template given parameters.""",
)
