#include "aos/configuration.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "gflags/gflags.h"

DEFINE_string(output_folder, "",
              "Name of folder to write the generated logfile to.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          aos::testing::ArtifactPath("aos/events/pingpong_config.json"));

  aos::SimulatedEventLoopFactory event_loop_factory(&config.message());

  // Event loop and app for Ping
  std::unique_ptr<aos::EventLoop> ping_event_loop =
      event_loop_factory.MakeEventLoop("ping");
  aos::Ping ping(ping_event_loop.get());

  // Event loop and app for Pong
  std::unique_ptr<aos::EventLoop> pong_event_loop =
      event_loop_factory.MakeEventLoop("pong");
  aos::Pong pong(pong_event_loop.get());

  std::unique_ptr<aos::EventLoop> log_writer_event_loop =
      event_loop_factory.MakeEventLoop("log_writer");
  aos::logger::Logger writer(log_writer_event_loop.get());
  writer.StartLoggingOnRun(FLAGS_output_folder);

  event_loop_factory.RunFor(std::chrono::seconds(10));
  return 0;
}
