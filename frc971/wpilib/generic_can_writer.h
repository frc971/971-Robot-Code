#include <map>
#include <string_view>

#include "frc971/can_configuration_generated.h"
#include "frc971/wpilib/falcon.h"
#include "frc971/wpilib/loop_output_handler.h"

namespace frc971 {
namespace wpilib {

/// This class uses a callback whenever it writes so that the caller can use any
/// flatbuffer to write to the falcon.
template <typename T>
class GenericCANWriter : public LoopOutputHandler<T> {
 public:
  GenericCANWriter(
      ::aos::EventLoop *event_loop,
      std::function<
          void(const T &output,
               std::map<std::string, std::shared_ptr<Falcon>> falcon_map)>
          write_callback)
      : LoopOutputHandler<T>(event_loop, "/superstructure"),
        write_callback_(write_callback) {
    event_loop->SetRuntimeRealtimePriority(kGenericCANWriterPriority);

    event_loop->OnRun([this]() { WriteConfigs(); });
  }

  void HandleCANConfiguration(const CANConfiguration &configuration) {
    for (auto &[_, falcon] : falcon_map_) {
      falcon->PrintConfigs();
    }

    if (configuration.reapply()) {
      WriteConfigs();
    }
  }

  void add_falcon(std::string_view name, std::shared_ptr<Falcon> falcon) {
    falcon_map_.insert({name, std::move(falcon)});
  }

  static constexpr int kGenericCANWriterPriority = 35;

 private:
  void WriteConfigs() {
    for (auto &[_, falcon] : falcon_map_) {
      falcon->WriteConfigs();
    }
  }

  void Write(const T &output) override { write_callback_(output, falcon_map_); }

  void Stop() override {
    AOS_LOG(WARNING, "Generic CAN output too old.\n");
    for (auto &[_, falcon] : falcon_map_) {
      falcon->WriteVoltage(0);
    }
  }

  // Maps each name to a falcon to let the caller retreive them when writing
  std::map<std::string_view, std::shared_ptr<Falcon>> falcon_map_;

  std::function<void(const T &output,
                     std::map<std::string, std::shared_ptr<Falcon>> falcon_map)>
      write_callback_;
};

}  // namespace wpilib
}  // namespace frc971
