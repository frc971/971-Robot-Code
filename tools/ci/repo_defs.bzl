def _ci_configure_impl(repository_ctx):
    """This repository rule tells other rules whether we're running in CI.

    Other rules can use this knowledge to make decisions about enforcing certain
    things on buildkite while relaxing restrictions during local development.
    """
    running_in_ci = repository_ctx.os.environ.get("FRC971_RUNNING_IN_CI", "0") == "1"
    repository_ctx.file("ci.bzl", """\
RUNNING_IN_CI = {}
""".format(running_in_ci))
    repository_ctx.file("BUILD", "")

ci_configure = repository_rule(
    implementation = _ci_configure_impl,
    environ = [
        # This is set in CI via tools/ci/buildkite.yaml.
        "FRC971_RUNNING_IN_CI",
    ],
)
