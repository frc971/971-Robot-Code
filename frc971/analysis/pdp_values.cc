#include <fstream>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "frc971/wpilib/pdp_values_generated.h"

ABSL_FLAG(std::string, output_path, "/tmp/pdp_values.csv", "");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));

  aos::SimulatedEventLoopFactory event_loop_factory(reader.configuration());

  reader.RegisterWithoutStarting(&event_loop_factory);

  const aos::Node *roborio =
      aos::configuration::GetNode(reader.configuration(), "roborio");

  std::unique_ptr<aos::EventLoop> event_loop =
      event_loop_factory.MakeEventLoop("roborio", roborio);

  std::ofstream file_stream;
  file_stream.open(absl::GetFlag(FLAGS_output_path));
  file_stream << "timestamp,currents,voltage\n";

  event_loop->SkipAosLog();
  event_loop->MakeWatcher(
      "/roborio/aos",
      [&file_stream, &event_loop](const frc971::PDPValues &pdp_values) {
        file_stream << event_loop->context().monotonic_event_time << ","
                    << "[";

        for (uint i = 0; i < pdp_values.currents()->size(); i++) {
          file_stream << pdp_values.currents()->Get(i);
          if (i != pdp_values.currents()->size() - 1) {
            file_stream << ", ";
          }
        }

        file_stream << "]," << pdp_values.voltage() << "\n";
      });

  event_loop_factory.Run();

  reader.Deregister();

  file_stream.close();

  return 0;
}
