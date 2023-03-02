def config_validator_rule(name, config, extension = ".bfbs", visibility = None):
    '''
    Macro to take a config and pass it to the config validator to validate that it will work on a real system.

    Currently just checks that the system can startup, but will check that timestamp channels are properly logged in the future.

    Args:
        name: name that the config validator uses, e.g. "test_config",
        config: config rule that needs to be validated, e.g. "//aos/events:pingpong_config",
    '''
    config_file = config + extension
    native.genrule(
        name = name,
        outs = [name + ".txt"],
        cmd = "$(location //aos/util:config_validator) --config $(location %s) > $@" % config_file,
        srcs = [config_file],
        tools = ["//aos/util:config_validator"],
        testonly = True,
        visibility = visibility,
    )
