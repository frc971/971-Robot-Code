#include <chrono>
#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"

DEFINE_string(config, "",
              "Name of the config file to replay using.");
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

  aos::SimulatedEventLoopFactory factory(&config.message());

  factory.RunFor(std::chrono::seconds(1));
}
