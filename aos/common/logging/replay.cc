#include "aos/common/logging/replay.h"

namespace aos {
namespace logging {
namespace linux_code {

bool LogReplayer::ProcessMessage() {
  const LogFileMessageHeader *message = reader_->ReadNextMessage(false);
  if (message == nullptr) return true;
  if (message->type != LogFileMessageHeader::MessageType::kStruct) return false;

  const char *position = reinterpret_cast<const char *>(message + 1);

  ::std::string process(position, message->name_size);
  position += message->name_size;

  uint32_t type_id;
  memcpy(&type_id, position, sizeof(type_id));
  position += sizeof(type_id);

  uint32_t message_length;
  memcpy(&message_length, position, sizeof(message_length));
  position += sizeof(message_length);
  ::std::string message_text(position, message_length);
  position += message_length;

  size_t split_index = message_text.find_first_of(':') + 2;
  split_index = message_text.find_first_of(':', split_index) + 2;
  message_text = message_text.substr(split_index);

  auto handler = handlers_.find(Key(process, message_text));
  if (handler == handlers_.end()) return false;

  handler->second->HandleStruct(
      ::aos::time::Time(message->time_sec, message->time_nsec), type_id,
      position,
      message->message_size -
          (sizeof(type_id) + sizeof(message_length) + message_length));
  return false;
}

}  // namespace linux_code
}  // namespace logging
}  // namespace aos
