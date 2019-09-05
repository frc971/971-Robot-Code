# Sourced from https://github.com/ribrdb/rules_emscripten/blob/master/toolchain/defs.bzl
# TODO(james): Specialize this more for our purposes--e.g.,
# we probably will only actually use one set of the possible options.
def emcc_binary(name,
                memory_init_file=0,
                wasm=True,
                worker=False,
                linkopts=[],
                **kwargs):
    includejs = False
    includehtml = False
    linkopts = list(linkopts)
    if name.endswith(".html"):
        basename = name[:-5]
        includehtml = True
        includejs = True
    elif name.endswith(".js"):
        basename = name[:-3]
        includejs = True
    outputs = []
    if includejs:
        outputs.append(basename + ".js")
        if wasm:
            outputs.append(basename + ".wasm")
        if memory_init_file:
            outputs.append(basename + ".mem")
        if worker:
            outputs.append(basename + ".worker.js")
            linkopts.append('--proxy-to-worker')

    if includehtml:
        outputs.append(basename + ".html")
    if not wasm:
        linkopts.append('-s WASM=0')
        linkopts.append('--memory-init-file %d' % memory_init_file)
    if includejs:
        tarfile = name + ".tar"
        # we'll generate a tarfile and extract multiple outputs
        native.cc_binary(name=tarfile, linkopts=linkopts, restricted_to = ["//tools:web"], **kwargs)
        native.genrule(
            name="emcc_extract_" + tarfile,
            srcs=[tarfile],
            outs=outputs,
            output_to_bindir=1,
            testonly=kwargs.get('testonly'),
            restricted_to = ["//tools:web"],
            cmd="""
          tar xf $< -C "$(@D)"/$$(dirname "%s")
        """ % [outputs[0]])
    else:
        native.cc_binary(name=name, linkopts=linkopts, restricted_to = ["//tools:web"], **kwargs)
