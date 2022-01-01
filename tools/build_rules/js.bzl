load("@build_bazel_rules_nodejs//:providers.bzl", "JSModuleInfo")
load("@npm//@bazel/rollup:index.bzl", upstream_rollup_bundle = "rollup_bundle")
load("@npm//@bazel/terser:index.bzl", "terser_minified")

def rollup_bundle(name, deps, visibility = None, **kwargs):
    """Calls the upstream rollup_bundle() and exposes a .min.js file.

    Legacy version of rollup_bundle() used to provide the .min.js file. This
    wrapper provides the same interface by explicitly exposing a .min.js file.
    """
    upstream_rollup_bundle(
        name = name,
        visibility = visibility,
        deps = deps + [
            "@npm//@rollup/plugin-node-resolve",
        ],
        config_file = "//:rollup.config.js",
        link_workspace_root = True,
        **kwargs
    )

    terser_minified(
        name = name + "__min",
        src = name + ".js",
    )

    # Copy the __min.js file (a declared output inside the rule) so that it's a
    # pre-declared output and publicly visible. I.e. via attr.output() below.
    _expose_minified_js(
        name = name + "__min_exposed",
        src = ":%s__min" % name,
        out = name + ".min.js",
        visibility = visibility,
    )

def _expose_minified_js_impl(ctx):
    """Copies the .min.js file in order to make it publicly accessible."""
    sources = ctx.attr.src[JSModuleInfo].sources.to_list()
    min_js = None
    for src in sources:
        if src.basename.endswith("__min.js"):
            min_js = src
            break

    if min_js == None:
        fail("Couldn't find .min.js in " + str(ctx.attr.src))

    ctx.actions.run(
        inputs = [min_js],
        outputs = [ctx.outputs.out],
        executable = "cp",
        arguments = [min_js.path, ctx.outputs.out.path],
    )

_expose_minified_js = rule(
    implementation = _expose_minified_js_impl,
    attrs = {
        "src": attr.label(providers = [JSModuleInfo]),
        "out": attr.output(mandatory = True),
    },
)
