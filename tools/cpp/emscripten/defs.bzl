def _emcc_expand_files_impl(ctx):
    tarfile = ctx.file.tarfile
    html_shell = ctx.file.html_shell
    basename = ctx.attr.name
    html_out = ctx.actions.declare_file(basename + ".html")
    tar_outs = [
        ctx.actions.declare_file(basename + "." + extension)
        for extension in ["js", "wasm"]
    ]
    if html_shell:
        ctx.actions.expand_template(
            output = html_out,
            template = html_shell,
            substitutions = {
                "{{{ SCRIPT }}}": "<script async type=\"text/javascript\" src=\"" + basename +
                                  ".js\"></script>",
            },
        )
    else:
        tar_outs.append(html_out)

    ctx.actions.run_shell(
        outputs = tar_outs,
        inputs = [tarfile],
        command = "tar xf " + tarfile.path + " -C \"" + html_out.dirname + "\"",
    )

    return [DefaultInfo(files = depset(tar_outs + [html_out]))]

emcc_expand_files = rule(
    attrs = {
        "html_shell": attr.label(
            mandatory = False,
            allow_single_file = True,
        ),
        "tarfile": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
    },
    doc = """
    Handles the intermediate processing to extra files from a tarball
    for emcc_binary. See emcc_binary for more detail.""",
    implementation = _emcc_expand_files_impl,
)

def emcc_binary(name, srcs = [], linkopts = [], html_shell = None, **kwargs):
    """Produces a deployable set of WebAssembly files.

    Depending on the settings, the exact format of the output varies.
    The output will be a a .js, .wasm, and optional .html file, all sharing the
    same basename. The .js file is the script that should be included in any
    webpage, and will handle calling the code compiled to the .wasm file.

    The optional .html file uses some existing template html file and adds the
    necessary <script> statement to import the .js script. This html file will
    be generated if the name of the rule ends with ".html"; if the html_shell
    argument is specified, then the provided html file is used to generate the
    output html. The only change made to the template html is to replace any
    instances of "{{{ SCRIPT }}}" with the appropriate <script> tags. This is
    consistent with how the "--shell-file" flag works in emscripten. However, we
    can't use the builtin flag with the script in its current form, because
    that would require making an html file an input to a cc_library rule,
    which bazel gets obnoxious about.

    This macro also defines a rule with a name equal to the basename of
    the name argument (e.g., if name = "foo.html", basename = "foo"). This rule
    is the rule that actually outputs the required files.

    Internally, this rule works by:
    1) Generating a tarball that contains the .js and .wasm files, using
       a cc_binary that calls the emscripten compiler.
    2) Extracting said tarball.
    3) [if necessary] Generating the output html from the html shell template.
    """
    includehtml = False
    linkopts = list(linkopts)
    srcs = list(srcs)
    if name.endswith(".html"):
        basename = name[:-5]
        includehtml = True
    elif name.endswith(".js"):
        basename = name[:-3]
    outputs = []
    outputs.append(basename + ".js")
    outputs.append(basename + ".wasm")

    if includehtml and not html_shell:
        outputs.append(basename + ".html")
    tarfile = name + ".tar"
    if html_shell:
        tarfile = basename + ".js.tar"
    native.cc_binary(
        name = tarfile,
        srcs = srcs,
        linkopts = linkopts,
        restricted_to = ["//tools:web"],
        **kwargs
    )
    emcc_expand_files(
        name = basename,
        html_shell = html_shell,
        tarfile = tarfile,
        restricted_to = ["//tools:web"],
    )
