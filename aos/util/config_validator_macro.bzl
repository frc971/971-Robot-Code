load("//tools/build_rules:clean_dep.bzl", "clean_dep")

def config_validator_test(name, config, logger_sets = [{}], check_for_not_logged_channels = False, validate_individual_node_loggers = False, extension = ".bfbs", visibility = None):
    '''
    Macro to take a config and pass it to the config validator to validate that it will work on a real system.

    Args:
        name: name that the config validator uses, e.g. "test_config",
        config: config rule that needs to be validated, e.g. "//aos/events:pingpong_config",
    '''
    config_file = config + extension
    config_json = json.encode({"logging": {"all_channels_logged": check_for_not_logged_channels, "validate_individual_node_loggers": validate_individual_node_loggers, "logger_sets": logger_sets}})
    native.cc_test(
        name = name,
        deps = [clean_dep("//aos/util:config_validator")],
        args = ["--config=$(location %s)" % config_file, "--validation_config='%s'" % config_json],
        data = [config_file],
        visibility = visibility,
    )
