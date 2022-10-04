def _mkdocs_impl(ctx):
    output_file = ctx.actions.declare_file(ctx.attr.name + ".tar")
    build_args = ctx.actions.args()
    build_args.add("build")
    site_subdir = "site"
    build_args.add("-f")
    build_args.add_all(ctx.files.config)
    build_args.add("--site-dir")
    build_args.add(site_subdir)
    site_dir = ctx.files.config[0].dirname + "/" + site_subdir
    ctx.actions.run(
        mnemonic = "MkDocsBuild",
        executable = ctx.executable._mkdocs,
        arguments = [build_args],
        inputs = ctx.files.srcs + ctx.files.config,
        env = {"OUTPUT": output_file.path, "SITE_DIR": site_dir},
        outputs = [output_file],
    )
    return [DefaultInfo(files = depset([output_file]))]

_mkdocs = rule(
    implementation = _mkdocs_impl,
    attrs = {
        "srcs": attr.label_list(
            allow_files = [".md"],
            doc = "A list of markdown files to generate docs for.",
        ),
        "config": attr.label(
            doc = "mkdocs.yaml configuration file to use for the documentation.",
            allow_files = [".yaml"],
            mandatory = True,
        ),
        "_mkdocs": attr.label(default = Label("@org_frc971//documentation:mkdocs_bin"), executable = True, cfg = "exec"),
    },
)

def mkdocs(name, srcs, config, **kwargs):
    """Bazel rule to build and serve mkdocs documentation.

    Build a tarball of the HTML files to be served by building "name", or get
    the functionality of `mkdocs serve` by doing a bazel run on "name.serve".

    Args:
      name: name of the rule.
      srcs: A list of markdown files to generate documentation from.
      config: A .yaml specifying the mkdocs configuration to use.
        See https://www.mkdocs.org/user-guide/configuration/
        Note that mkdocs does not allow the mkdocs.yaml configuration to be
        in the same folder as the markdown files (canonically, if the
        mkdocs.yaml is at foo/mkdocs.yaml, the markdown files will be
        at foo/docs/*.md).
    """
    _mkdocs(name = name, srcs = srcs, config = config, **kwargs)
    native.py_binary(
        name = name + ".serve",
        srcs = ["@org_frc971//documentation:mkdocs_bin.py"],
        main = "mkdocs_bin.py",
        args = ["serve", "-f", "$(location %s)" % (config,)],
        deps = ["@pip//mkdocs"],
        data = srcs + [config],
        **kwargs
    )
