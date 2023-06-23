load("//tools/build_rules:label.bzl", "expand_label")
load("//tools/build_rules:select.bzl", "address_size_select")

AosConfigInfo = provider(fields = [
    "transitive_flatbuffers",
    "transitive_src",
])

def aos_config(name, src, flatbuffers = [], deps = [], visibility = None, testonly = False, target_compatible_with = None):
    _aos_config(
        name = name,
        src = src,
        flags = address_size_select({
            "32": ["--max_queue_size_override=0xffff"],
            "64": ["--max_queue_size_override=0xffffffff"],
        }),
        config_json = name + ".json",
        config_stripped = name + ".stripped.json",
        config_binary = name + ".bfbs",
        deps = deps,
        flatbuffers = [expand_label(flatbuffer) + "_reflection_out" for flatbuffer in flatbuffers],
        visibility = visibility,
        testonly = testonly,
        target_compatible_with = target_compatible_with,
    )

def _aos_config_impl(ctx):
    config = ctx.outputs.config_json
    stripped_config = ctx.outputs.config_stripped
    binary_config = ctx.outputs.config_binary

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
        arguments = ctx.attr.flags + [
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
        "config_json": attr.output(mandatory = True),
        "config_stripped": attr.output(mandatory = True),
        "config_binary": attr.output(mandatory = True),
        "_config_flattener": attr.label(
            executable = True,
            cfg = "host",
            default = Label("//aos:config_flattener"),
        ),
        "src": attr.label(
            mandatory = True,
            allow_files = True,
        ),
        "flags": attr.string_list(
            doc = "Additional flags to pass to config_flattener.",
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
