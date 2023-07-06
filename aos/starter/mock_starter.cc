#include "aos/starter/mock_starter.h"

namespace aos {
namespace starter {

MockStarter::MockStarter(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      status_sender_(event_loop_->MakeSender<aos::starter::Status>("/aos")) {
  aos::TimerHandler *send_timer =
      event_loop_->AddTimer([this]() { SendStatus(); });

  CHECK(aos::configuration::MultiNode(event_loop_->configuration()));

  for (const aos::Node *node :
       aos::configuration::GetNodes(event_loop_->configuration())) {
    const aos::Channel *channel = aos::starter::StarterRpcChannelForNode(
        event_loop_->configuration(), node);
    if (aos::configuration::ChannelIsReadableOnNode(channel,
                                                    event_loop_->node())) {
      std::string_view channel_name = channel->name()->string_view();
      event_loop_->MakeWatcher(
          channel_name, [this](const aos::starter::StarterRpc &command) {
            for (const flatbuffers::String *node : *command.nodes()) {
              if (node->string_view() ==
                  event_loop_->node()->name()->string_view()) {
                CHECK(statuses_.count(command.name()->str()) > 0)
                    << "Unable to find " << command.name()->string_view()
                    << " in our list of applications.";
                ApplicationStatus &status = statuses_[command.name()->str()];
                switch (command.command()) {
                  case aos::starter::Command::START:
                    if (!status.running) {
                      VLOG(1) << "Starting " << command.name()->string_view()
                              << " at " << event_loop_->monotonic_now();
                      status.running = true;
                      status.start_time = event_loop_->monotonic_now();
                      status.id = next_id_++;
                    }
                    break;
                  case aos::starter::Command::STOP:
                    if (status.running) {
                      VLOG(1) << "Stopping " << command.name()->string_view()
                              << " at " << event_loop_->monotonic_now();
                    }
                    status.running = false;
                    break;
                  case aos::starter::Command::RESTART:
                    status.running = true;
                    VLOG(1) << "Restarting " << command.name()->string_view()
                            << " at " << event_loop_->monotonic_now();
                    status.start_time = event_loop_->monotonic_now();
                    status.id = next_id_++;
                }
                SendStatus();
              }
            }
          });
    }
  }

  event_loop_->OnRun([this, send_timer]() {
    send_timer->Schedule(event_loop_->monotonic_now(), std::chrono::seconds(1));

    for (const aos::Application *application :
         *event_loop_->configuration()->applications()) {
      if (aos::configuration::ApplicationShouldStart(
              event_loop_->configuration(), event_loop_->node(), application)) {
        statuses_[application->name()->str()] = ApplicationStatus{
            next_id_++, application->autostart(), event_loop_->monotonic_now()};
      }
    }
  });
}

void MockStarter::SendStatus() {
  aos::Sender<aos::starter::Status>::Builder builder =
      status_sender_.MakeBuilder();
  std::vector<flatbuffers::Offset<aos::starter::ApplicationStatus>>
      status_offsets;
  for (const std::pair<const std::string, ApplicationStatus> &pair :
       statuses_) {
    const flatbuffers::Offset<flatbuffers::String> name_offset =
        builder.fbb()->CreateString(pair.first);
    aos::starter::ApplicationStatus::Builder status_builder =
        builder.MakeBuilder<aos::starter::ApplicationStatus>();
    status_builder.add_name(name_offset);
    status_builder.add_state(pair.second.running
                                 ? aos::starter::State::RUNNING
                                 : aos::starter::State::STOPPED);
    status_builder.add_last_exit_code(0);
    status_builder.add_id(pair.second.id);
    status_builder.add_last_stop_reason(
        aos::starter::LastStopReason::STOP_REQUESTED);
    status_builder.add_last_start_time(
        pair.second.start_time.time_since_epoch().count());
    if (pair.second.running) {
      status_builder.add_pid(pair.second.id);
    }
    status_offsets.push_back(status_builder.Finish());
  }
  const flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<aos::starter::ApplicationStatus>>>
      statuses_offset = builder.fbb()->CreateVector(status_offsets);
  aos::starter::Status::Builder status_builder =
      builder.MakeBuilder<aos::starter::Status>();
  status_builder.add_statuses(statuses_offset);
  builder.CheckOk(builder.Send(status_builder.Finish()));
}

MockStarters::MockStarters(aos::SimulatedEventLoopFactory *event_loop_factory) {
  CHECK(aos::configuration::MultiNode(event_loop_factory->configuration()));
  for (const aos::Node *node :
       aos::configuration::GetNodes(event_loop_factory->configuration())) {
    event_loops_.emplace_back(
        event_loop_factory->GetNodeEventLoopFactory(node)->MakeEventLoop(
            "starterd"));
    mock_starters_.emplace_back(
        std::make_unique<MockStarter>(event_loops_.back().get()));
  }
}

}  // namespace starter
}  // namespace aos
