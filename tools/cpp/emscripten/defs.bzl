# Sourced from https://github.com/ribrdb/rules_emscripten/blob/master/toolchain/defs.bzl
# TODO(james): Specialize this more for our purposes--e.g.,
# we probably will only actually use one set of the possible options.
def emcc_binary(name, srcs=[], linkopts=[], html_shell=None, **kwargs):
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
    TODO(james): Rewrite this as a rule so that we can do some of this more
    cleanly.

    This macro also defines a rule with a name equal to the basename of
    the name argument (e.g., if name = "foo.html", basename = "foo"). This rule
    is a filegroup containing all the output files of this rule.

    Internally, this rule works by:
    1) Generating a tarball that contains the .js and .wasm files, using
       a cc_binary that calls the emscripten compiler.
    2) Extracting said tarball.
    3) Generating the output html from the html shell template.
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
        name=tarfile,
        srcs=srcs,
        linkopts=linkopts,
        restricted_to = ["//tools:web"],
        **kwargs)
    native.genrule(
        name="emcc_extract_" + tarfile,
        srcs=[tarfile],
        outs=outputs,
        output_to_bindir=1,
        testonly=kwargs.get('testonly'),
        restricted_to = ["//tools:web"],
        cmd="""tar xf $< -C "$(@D)"/$$(dirname "%s")""" % [outputs[0]])
    if html_shell:
         native.genrule(
             name = "generate_shell_" + name,
             srcs = [html_shell],
             outs = [basename + ".html"],
             restricted_to = ["//tools:web"],
             cmd = "sed 's/{{{ SCRIPT }}}/<script async type=\"text\/javascript\" src=\"" + basename + ".js\"><\/script>/' $< > $@",
         )
         outputs.append(basename + ".html")
    native.filegroup(
        name = basename,
        srcs = outputs,
        restricted_to = ["//tools:web"]
    )
