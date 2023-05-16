def config_validator_rule(name, config, logger_sets = [{}], check_for_not_logged_channels = False, extension = ".bfbs", visibility = None):
    '''
    Macro to take a config and pass it to the config validator to validate that it will work on a real system.

    Currently just checks that the system can startup, but will check that timestamp channels are properly logged in the future.

    Args:
        name: name that the config validator uses, e.g. "test_config",
        config: config rule that needs to be validated, e.g. "//aos/events:pingpong_config",
    '''
    config_file = config + extension
    config_json = json.encode({"logging": {"all_channels_logged": check_for_not_logged_channels, "logger_sets": logger_sets}})
    native.genrule(
        name = name,
        outs = [name + ".txt"],
        cmd = "$(location //aos/util:config_validator) --config $(location %s) --validation_config='%s' && touch $@" % (config_file, config_json),
        srcs = [config_file],
        tools = ["//aos/util:config_validator"],
        testonly = True,
        visibility = visibility,
    )
