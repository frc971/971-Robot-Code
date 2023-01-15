#ifndef FRC971_CONSTANTS_CONSTANTS_SENDER_H_
#define FRC971_CONSTANTS_CONSTANTS_SENDER_H_

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

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
    event_loop->OnRun([this]() {
      typename aos::Sender<ConstantsData>::Builder builder =
          sender_.MakeBuilder();
      builder.CheckOk(builder.Send(GetConstantsForTeamNumber(builder.fbb())));
    });
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

}  // namespace frc971::constants

#endif  // FRC971_CONSTANTS_CONSTANTS_SENDER_H_
