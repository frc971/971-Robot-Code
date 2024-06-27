#include "aos/events/simple_channel.h"

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

namespace aos {

SimpleChannel::SimpleChannel(const Channel *channel) {
  CHECK(channel != nullptr);
  const flatbuffers::String *channel_name = channel->name();
  CHECK(channel_name != nullptr);
  name = channel_name->str();
  const flatbuffers::String *channel_type = channel->type();
  CHECK(channel_type != nullptr);
  type = channel_type->str();
}

std::string SimpleChannel::DebugString() const {
  return absl::StrCat("{ ", name, ", ", type, "}");
}

bool SimpleChannel::operator==(const SimpleChannel &other) const {
  return name == other.name && type == other.type;
}

bool SimpleChannel::operator<(const SimpleChannel &other) const {
  int name_compare = other.name.compare(name);
  if (name_compare == 0) {
    return other.type < type;
  } else if (name_compare < 0) {
    return true;
  } else {
    return false;
  }
}

}  // namespace aos
