def _collect_nodejs_headers_impl(ctx):
    """Collect all the headers for compiling a native node module.

    The @nodejs_linux_amd64 repo doesn't expose a dedicated target for the
    nodejs headers. This rule will collect all the necessary headers. Add a
    target of this rule to `srcs` of a `cc_binary` target.
    """
    files = ctx.attr.src.files.to_list()
    headers = []
    for file in files:
        _, _, header = file.short_path.partition("/bin/nodejs/include/node/")
        if not header or not header.endswith(".h"):
            continue
        if "/" not in header:
            headers.append(file)
        elif header.startswith("cppgc/"):
            headers.append(file)

    return [DefaultInfo(
        files = depset(headers),
    )]

collect_nodejs_headers = rule(
    implementation = _collect_nodejs_headers_impl,
    attrs = {
        "src": attr.label(),
    },
)
