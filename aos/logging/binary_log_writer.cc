#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <mntent.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <string>

#include <chrono>
#include <map>
#include <unordered_set>

#include "aos/die.h"
#include "aos/logging/binary_log_file.h"
#include "aos/logging/implementations.h"
#include "aos/logging/log_namer.h"
#include "aos/time/time.h"
#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/ipc_lib/queue.h"

namespace aos {
namespace logging {
namespace linux_code {
namespace {

int BinaryLogReaderMain() {
  InitNRT();

  const std::string file_name = GetLogName("aos_log");
  int fd = open(file_name.c_str(), O_SYNC | O_APPEND | O_RDWR | O_CREAT,
                S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
  if (fd == -1) {
    AOS_PLOG(FATAL, "opening file '%s' failed", file_name.c_str());
  }
  LogFileWriter writer(fd);

  RawQueue *queue = GetLoggingQueue();

  ::std::unordered_set<uint32_t> written_type_ids;

  while (true) {
    const LogMessage *const msg =
        static_cast<const LogMessage *>(queue->ReadMessage(RawQueue::kNonBlock));
    if (msg == NULL) {
      // If we've emptied the queue, then wait for a bit before starting to read
      // again so the queue can buffer up some logs. This avoids lots of context
      // switches and mutex contention which happens if we're constantly reading
      // new messages as they come in.
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(100));
      continue;
    }

    const size_t raw_output_length =
        sizeof(LogFileMessageHeader) + msg->name_length + msg->message_length;
    size_t output_length = raw_output_length;
    LogFileMessageHeader *const output = writer.GetWritePosition(output_length);
    char *output_strings = reinterpret_cast<char *>(output) + sizeof(*output);
    output->name_size = msg->name_length;
    output->message_size = msg->message_length;
    output->source = msg->source;
    output->level = msg->level;
    output->time_sec = msg->seconds;
    output->time_nsec = msg->nseconds;
    output->sequence = msg->sequence;
    memcpy(output_strings, msg->name, msg->name_length);

    switch (msg->type) {
      case LogMessage::Type::kString:
        memcpy(output_strings + msg->name_length, msg->message,
               msg->message_length);
        output->type = LogFileMessageHeader::MessageType::kString;
        break;
    }

    if (output->message_size - msg->message_length !=
        output_length - raw_output_length) {
      AOS_LOG(FATAL, "%zu != %zu\n", output->message_size - msg->message_length,
              output_length - raw_output_length);
    }

    futex_set(&output->marker);

    queue->FreeMessage(msg);
  }

  Cleanup();
  return 0;
}

}  // namespace
}  // namespace linux_code
}  // namespace logging
}  // namespace aos

int main() {
  return ::aos::logging::linux_code::BinaryLogReaderMain();
}
