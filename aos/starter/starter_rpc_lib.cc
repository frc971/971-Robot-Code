#include "starter_rpc_lib.h"

#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"

namespace aos {
namespace starter {

const aos::starter::ApplicationStatus *FindApplicationStatus(
    const aos::starter::Status &status, std::string_view name) {
  if (!status.has_statuses()) {
    return nullptr;
  }

  auto statuses = status.statuses();

  auto search =
      std::find_if(statuses->begin(), statuses->end(),
                   [name](const aos::starter::ApplicationStatus *app_status) {
                     return app_status->has_name() &&
                            app_status->name()->string_view() == name;
                   });
  if (search == statuses->end()) {
    return nullptr;
  }
  return *search;
}

std::string_view FindApplication(const std::string_view &name,
                                 const aos::Configuration *config) {
  std::string_view app_name = name;
  for (const auto app : *config->applications()) {
    if (app->has_executable_name() &&
        app->executable_name()->string_view() == name) {
      app_name = app->name()->string_view();
      break;
    }
  }
  return app_name;
}

bool SendCommandBlocking(aos::starter::Command command, std::string_view name,
                         const aos::Configuration *config,
                         std::chrono::milliseconds timeout) {
  aos::ShmEventLoop event_loop(config);
  event_loop.SkipAosLog();

  ::aos::Sender<aos::starter::StarterRpc> cmd_sender =
      event_loop.MakeSender<aos::starter::StarterRpc>("/aos");

  // Wait until event loop starts to send command so watcher is ready
  event_loop.OnRun([&cmd_sender, command, name] {
    aos::Sender<aos::starter::StarterRpc>::Builder builder =
        cmd_sender.MakeBuilder();

    auto name_str = builder.fbb()->CreateString(name);

    aos::starter::StarterRpc::Builder cmd_builder =
        builder.MakeBuilder<aos::starter::StarterRpc>();

    cmd_builder.add_name(name_str);
    cmd_builder.add_command(command);

    builder.Send(cmd_builder.Finish());
  });

  // If still waiting after timeout milliseconds, exit the loop
  event_loop.AddTimer([&event_loop] { event_loop.Exit(); })
      ->Setup(event_loop.monotonic_now() + timeout);

  // Fetch the last list of statuses to compare the requested application's id
  // against for commands such as restart.
  auto initial_status_fetcher =
      event_loop.MakeFetcher<aos::starter::Status>("/aos");
  initial_status_fetcher.Fetch();
  auto initial_status =
      initial_status_fetcher.get()
          ? FindApplicationStatus(*initial_status_fetcher, name)
          : nullptr;

  const std::optional<uint64_t> initial_id =
      (initial_status != nullptr && initial_status->has_id())
          ? std::make_optional(initial_status->id())
          : std::nullopt;

  bool success = false;
  event_loop.MakeWatcher(
      "/aos", [&event_loop, command, name, initial_id,
               &success](const aos::starter::Status &status) {
        const aos::starter::ApplicationStatus *app_status =
            FindApplicationStatus(status, name);

        const std::optional<aos::starter::State> state =
            (app_status != nullptr && app_status->has_state())
                ? std::make_optional(app_status->state())
                : std::nullopt;

        switch (command) {
          case aos::starter::Command::START: {
            if (state == aos::starter::State::RUNNING) {
              success = true;
              event_loop.Exit();
            }
            break;
          }
          case aos::starter::Command::STOP: {
            if (state == aos::starter::State::STOPPED) {
              success = true;
              event_loop.Exit();
            }
            break;
          }
          case aos::starter::Command::RESTART: {
            if (state == aos::starter::State::RUNNING && app_status->has_id() &&
                app_status->id() != initial_id) {
              success = true;
              event_loop.Exit();
            }
            break;
          }
        }
      });

  event_loop.Run();

  return success;
}

const FlatbufferDetachedBuffer<aos::starter::ApplicationStatus> GetStatus(
    std::string_view name, const Configuration *config) {
  ShmEventLoop event_loop(config);
  event_loop.SkipAosLog();

  auto status_fetcher = event_loop.MakeFetcher<aos::starter::Status>("/aos");
  status_fetcher.Fetch();
  auto status =
      status_fetcher ? FindApplicationStatus(*status_fetcher, name) : nullptr;
  return status ? aos::CopyFlatBuffer(status)
                : FlatbufferDetachedBuffer<
                      aos::starter::ApplicationStatus>::Empty();
}

std::optional<const aos::FlatbufferVector<aos::starter::Status>>
GetStarterStatus(const aos::Configuration *config) {
  ShmEventLoop event_loop(config);
  event_loop.SkipAosLog();

  auto status_fetcher = event_loop.MakeFetcher<aos::starter::Status>("/aos");
  status_fetcher.Fetch();
  return (status_fetcher ? std::make_optional(status_fetcher.CopyFlatBuffer())
                         : std::nullopt);
}

}  // namespace starter
}  // namespace aos
