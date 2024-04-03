def _jinja2_template_impl(ctx):
    out = ctx.outputs.out
    parameters = dict(ctx.attr.parameters)
    parameters.update(ctx.attr.list_parameters)

    # For now we don't really want the user to worry about which configuration
    # to pull the file from. We don't yet have a use case for pulling the same
    # file from multiple configurations. We point Jinja at all the configuration
    # roots.
    include_dirs = depset([
        file.root.path or "."
        for file in ctx.files.includes
    ]).to_list()

    args = ctx.actions.args()
    args.add(ctx.file.src)
    args.add(json.encode(parameters))
    args.add(out)
    args.add_all(include_dirs, before_each = "--include_dir")

    ctx.actions.run(
        inputs = ctx.files.src + ctx.files.includes,
        tools = [ctx.executable._jinja2],
        progress_message = "Generating " + out.short_path,
        outputs = [out],
        executable = ctx.executable._jinja2,
        arguments = [args],
    )

    return [DefaultInfo(files = depset([out])), OutputGroupInfo(out = depset([out]))]

jinja2_template_rule = rule(
    attrs = {
        "out": attr.output(
            mandatory = True,
            doc = """The file to generate using the template. If using the jinja2_template macro below, this will automatically be populated with the contents of the `name` parameter.""",
        ),
        "src": attr.label(
            mandatory = True,
            allow_single_file = True,
            doc = """The jinja2 template file to expand.""",
        ),
        "parameters": attr.string_dict(
            mandatory = False,
            default = {},
            doc = """The string parameters to supply to Jinja2.""",
        ),
        "list_parameters": attr.string_list_dict(
            mandatory = False,
            default = {},
            doc = """The string list parameters to supply to Jinja2.""",
        ),
        "includes": attr.label_list(
            allow_files = True,
            doc = """Files which are included by the template.""",
        ),
        "_jinja2": attr.label(
            default = "//tools/build_rules:jinja2_generator",
            cfg = "exec",
            executable = True,
        ),
    },
    implementation = _jinja2_template_impl,
    doc = """Expands a jinja2 template given parameters.""",
)

def jinja2_template(name, src, parameters = {}, list_parameters = {}, **kwargs):
    # Since the `out` field will be set to `name`, and the name for the rule must
    # differ from `out`, name the rule as the `name` plus a suffix
    rule_name = name + "_rule"

    jinja2_template_rule(
        name = rule_name,
        out = name,
        src = src,
        parameters = parameters,
        list_parameters = list_parameters,
        **kwargs
    )
