#include "aos/events/event_loop.h"

#include "aos/configuration.h"
#include "aos/configuration_generated.h"
#include "glog/logging.h"

namespace aos {

void EventLoop::ValidateChannel(const Channel *channel) {
  CHECK(configuration_->channels() != nullptr) << ": No channels";

  CHECK(std::find(configuration_->channels()->begin(),
                  configuration_->channels()->end(),
                  channel) != configuration_->channels()->end())
      << ": Channel pointer not found in configuration()->channels()";
}

}  // namespace aos
