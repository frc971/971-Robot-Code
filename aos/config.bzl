load("//tools/build_rules:label.bzl", "expand_label")

AosConfigInfo = provider(fields = [
    "transitive_flatbuffers",
    "transitive_src",
])

def aos_config(name, src, flatbuffers = [], deps = [], visibility = None, testonly = False):
    _aos_config(
        name = name,
        src = src,
        deps = deps,
        flatbuffers = [expand_label(flatbuffer) + "_reflection_out" for flatbuffer in flatbuffers],
        visibility = visibility,
        testonly = testonly,
    )

def _aos_config_impl(ctx):
    config = ctx.actions.declare_file(ctx.label.name + ".json")
    stripped_config = ctx.actions.declare_file(ctx.label.name + ".stripped.json")
    binary_config = ctx.actions.declare_file(ctx.label.name + ".bfbs")

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
        outputs = [config, stripped_config, binary_config],
        inputs = all_files,
        arguments = [
            config.path,
            stripped_config.path,
            binary_config.path,
            (ctx.label.workspace_root or ".") + "/" + ctx.files.src[0].short_path,
            ctx.bin_dir.path,
        ] + [f.path for f in flatbuffers_depset.to_list()],
        progress_message = "Flattening config",
        executable = ctx.executable._config_flattener,
    )
    runfiles = ctx.runfiles(files = [config, stripped_config, binary_config])
    return [
        DefaultInfo(
            files = depset([config, stripped_config, binary_config]),
            runfiles = runfiles,
        ),
        AosConfigInfo(
            transitive_flatbuffers = flatbuffers_depset,
            transitive_src = src_depset,
        ),
    ]

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
    implementation = _aos_config_impl,
)
