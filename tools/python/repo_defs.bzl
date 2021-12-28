def _pip_configure_impl(repository_ctx):
    """Runs tools/python/pip_configure.py."""
    script_path = repository_ctx.path(repository_ctx.attr._script).realpath
    interpreter_path = repository_ctx.path(repository_ctx.attr._interpreter).realpath
    requirements_path = repository_ctx.path(repository_ctx.attr._requirements).realpath

    script_result = repository_ctx.execute([
        interpreter_path,
        "-BSs",
        script_path,
        requirements_path,
    ])
    if script_result.return_code != 0:
        fail("{} failed: {} ({})".format(
            script_path,
            script_result.stdout,
            script_result.stderr,
        ))

pip_configure = repository_rule(
    implementation = _pip_configure_impl,
    attrs = {
        "_interpreter": attr.label(
            default = "@python3_9_x86_64-unknown-linux-gnu//:bin/python3",
        ),
        "_script": attr.label(
            default = "@//tools/python:pip_configure.py",
        ),
        "_requirements": attr.label(
            default = "@//tools/python:requirements.txt",
        ),
    },
)
