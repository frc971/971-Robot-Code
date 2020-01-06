load("//tools/build_rules:label.bzl", "expand_label")

AosConfigInfo = provider(fields = ["transitive_flatbuffers", "transitive_src"])

def aos_config(name, src, flatbuffers = [], deps = [], visibility = None):
    _aos_config(
        name = name,
        src = src,
        deps = deps,
        flatbuffers = [expand_label(flatbuffer) + "_reflection_out" for flatbuffer in flatbuffers],
        visibility = visibility,
    )

def _aos_config_impl(ctx):
    flatbuffers_depset = depset(
        ctx.files.flatbuffers,
        transitive = [dep[AosConfigInfo].transitive_flatbuffers for dep in ctx.attr.deps],
    )

    src_depset = depset(
        ctx.files.src,
        transitive = [dep[AosConfigInfo].transitive_src for dep in ctx.attr.deps],
    )

    all_files = flatbuffers_depset.to_list() + src_depset.to_list()
    ctx.actions.run(
        outputs = [ctx.outputs.config],
        inputs = all_files,
        arguments = [ctx.outputs.config.path, ctx.files.src[0].path] + [f.path for f in flatbuffers_depset.to_list()],
        progress_message = "Flattening config",
        executable = ctx.executable._config_flattener,
    )
    return AosConfigInfo(
        transitive_flatbuffers = flatbuffers_depset,
        transitive_src = src_depset,
    )

_aos_config = rule(
    attrs = {
        "_config_flattener": attr.label(
            executable = True,
            cfg = "host",
            default = Label("//aos:config_flattener"),
        ),
        "src": attr.label(
            mandatory = True,
            allow_files = True,
        ),
        "deps": attr.label_list(
            providers = [AosConfigInfo],
        ),
        "flatbuffers": attr.label_list(
            mandatory = False,
        ),
    },
    outputs = {
        "config": "%{name}.json",
    },
    implementation = _aos_config_impl,
)
