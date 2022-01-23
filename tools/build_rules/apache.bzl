def _apache_binary_impl(ctx):
    binary_path = ctx.attr.binary.files_to_run.executable.short_path

    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, """\
#!/bin/bash

exec ./tools/build_rules/apache_runner --binary "{}" "$@"
""".format(binary_path), is_executable = True)

    # Collect files and runfiles for the tools that we're wrapping.
    files = depset(transitive = [
        ctx.attr._apache_runner.files,
        ctx.attr.binary.files,
    ])

    runfiles = ctx.attr._apache_runner.default_runfiles
    runfiles = runfiles.merge(ctx.attr.binary.default_runfiles)

    return [
        DefaultInfo(
            executable = out,
            files = files,
            runfiles = runfiles,
        ),
    ]

apache_wrapper = rule(
    implementation = _apache_binary_impl,
    attrs = {
        "binary": attr.label(
            mandatory = True,
            executable = True,
            cfg = "target",
            doc = "The binary that we're wrapping with LDAP+HTTPS.",
        ),
        "_apache_runner": attr.label(
            default = "@//tools/build_rules:apache_runner",
            executable = True,
            cfg = "target",
        ),
    },
    doc = """\
This rule wraps another web server and provides LDAP and HTTPS support.

It's not intended to be used in production. It's intended to provide team
members with a way to test their code in a production-like environment. E.g. to
test whether your server makes use of LDAP credentials correctly.

Write this as a wrapper around another binary like so:

    apache_wrapper(
        name = "wrapped_server",
        binary = "//path/to:server_binary",
    )

Then you can run Apache and the wrapped binary like so:

    $ bazel run :wrapped_server

The wrapped binary can find the port that Apache is wrapping via the
APACHE_WRAPPED_PORT environment variable.

This rule assumes that you have a file at the root of the workspace called
"ldap.json". You can customize this path with the `--ldap_info` argument. The
JSON file has to have these three entries in it:

    {
        "ldap_bind_dn": "...",
        "ldap_url": "...",
        "ldap_password": "..."
    }

where the "..." values are replaced with the information to connect to an LDAP
server. If you want to connect to our FRC971 LDAP server, please contact a
Software mentor. Or ask on the `#coding` Slack channel.

If the default ports of 7000 and 7500 are already taken, you can change them via
the `--https_port` and `--wrapped_port` arguments.
""",
    executable = True,
)
