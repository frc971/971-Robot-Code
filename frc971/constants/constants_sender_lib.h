#ifndef FRC971_CONSTANTS_CONSTANTS_SENDER_H_
#define FRC971_CONSTANTS_CONSTANTS_SENDER_H_

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"

namespace frc971::constants {

// Publishes the constants specific to the current robot
template <typename ConstantsData, typename ConstantsList>
class ConstantSender {
 public:
  ConstantSender(aos::EventLoop *event_loop, std::string constants_path,
                 std::string_view channel_name = "/constants")
      : ConstantSender<ConstantsData, ConstantsList>(
            event_loop, constants_path, aos::network::GetTeamNumber(),
            channel_name) {}

  ConstantSender(aos::EventLoop *event_loop, std::string constants_path,
                 const uint16_t team_number, std::string_view channel_name)
      : team_number_(team_number),
        channel_name_(channel_name),
        constants_path_(constants_path),
        event_loop_(event_loop),
        sender_(event_loop_->MakeSender<ConstantsData>(channel_name_)) {
    typename aos::Sender<ConstantsData>::Builder builder =
        sender_.MakeBuilder();
    builder.CheckOk(builder.Send(GetConstantsForTeamNumber(builder.fbb())));
  }

 private:
  const uint16_t team_number_ = 0;
  std::string_view channel_name_;
  flatbuffers::Offset<ConstantsData> GetConstantsForTeamNumber(
      flatbuffers::FlatBufferBuilder *fbb) {
    aos::FlatbufferDetachedBuffer<ConstantsList> fb =
        aos::JsonFileToFlatbuffer<ConstantsList>(constants_path_);
    const ConstantsList &message = fb.message();
    const auto *constants = message.constants();
    // Search through the constants for the one matching our team number.
    for (const auto &constant_data : *constants) {
      if (team_number_ == constant_data->team()) {
        // Values is equal to the constants meant for the specific robot.
        const ConstantsData *values = constant_data->data();
        flatbuffers::Offset<ConstantsData> flatbuffer_constants =
            aos::RecursiveCopyFlatBuffer(values, fbb);
        return flatbuffer_constants;
      }
    }
    LOG(FATAL) << "There was no match for " << team_number_
               << ". Check the constants.json file for the team number that is "
                  "missing.";
  }

  std::string constants_path_;
  aos::EventLoop *event_loop_;
  aos::Sender<ConstantsData> sender_;
};

// This class fetches the current constants for the device, with appropriate
// CHECKs to ensure that (a) the constants never change and (b) that the
// constants are always available. This can be paired with WaitForConstants to
// create the conditions for (b). In simulation, the constants should simply be
// sent before starting up other EventLoops.
template <typename ConstantsData>
class ConstantsFetcher {
 public:
  ConstantsFetcher(aos::EventLoop *event_loop,
                   std::string_view channel = "/constants")
      : fetcher_(event_loop->MakeFetcher<ConstantsData>(channel)) {
    CHECK(fetcher_.Fetch())
        << "Constants information must be available at startup.";
    event_loop->MakeNoArgWatcher<ConstantsData>(channel, []() {
      LOG(FATAL)
          << "Don't know how to handle changes to constants information.";
    });
  }

  const ConstantsData &constants() const { return *fetcher_.get(); }

 private:
  aos::Fetcher<ConstantsData> fetcher_;
};

// Blocks until data is available on the requested channel using a ShmEventLoop.
// This is for use during initialization in C++ binaries so that we can delay
// initialization until everything is available. This allows applications to
// depend on constants data during their initialization.
template <typename ConstantsData>
void WaitForConstants(const aos::Configuration *config,
                      std::string_view channel = "/constants") {
  aos::ShmEventLoop event_loop(config);
  aos::Fetcher fetcher = event_loop.MakeFetcher<ConstantsData>(channel);
  event_loop.MakeNoArgWatcher<ConstantsData>(
      channel, [&event_loop]() { event_loop.Exit(); });
  event_loop.OnRun([&event_loop, &fetcher]() {
    // If the constants were already published, we don't need to wait for them.
    if (fetcher.Fetch()) {
      event_loop.Exit();
    }
  });
  LOG(INFO) << "Waiting for constants data on " << channel << " "
            << ConstantsData::GetFullyQualifiedName();
  event_loop.Run();
  LOG(INFO) << "Got constants data.";
}

}  // namespace frc971::constants

#endif  // FRC971_CONSTANTS_CONSTANTS_SENDER_H_
