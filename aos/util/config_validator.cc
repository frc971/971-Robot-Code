#include "aos/json_to_flatbuffer.h"
#include "aos/util/config_validator_lib.h"

DEFINE_string(config, "", "Name of the config file to replay using.");
DEFINE_string(validation_config, "{}",
              "JSON config to use to validate the config.");
/* This binary is used to validate that all of the
   needed remote timestamps channels are in the config
   to log the timestamps.
   Future versions of the validator will provide the option
   to confirm that the timestamps in the config are able to
   replay all of the data in the log
   This can be done by getting a list of all of the nodes and
   iterating through it with a for loop creating a logger for
   each one
   Reference superstructure_lib_test.cc*/
TEST(ConfigValidatorTest, ReadConfig) {
  ASSERT_TRUE(!FLAGS_config.empty());
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  const aos::FlatbufferDetachedBuffer<aos::util::ConfigValidatorConfig>
      validator_config =
          aos::JsonToFlatbuffer<aos::util::ConfigValidatorConfig>(
              FLAGS_validation_config);
  aos::util::ConfigIsValid(&config.message(), &validator_config.message());
}

// TODO(milind): add more tests, the above one doesn't
// catch an error like forgetting to add a forwarded message to
// the destination node's config.
