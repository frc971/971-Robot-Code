#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <fcntl.h>
#include <dirent.h>
#include <mntent.h>

#include <map>
#include <unordered_set>

#include "aos/common/logging/implementations.h"
#include "aos/common/logging/binary_log_file.h"
#include "aos/linux_code/init.h"
#include "aos/linux_code/configuration.h"
#include "aos/linux_code/ipc_lib/queue.h"
#include "aos/common/queue_types.h"
#include "aos/common/die.h"
#include "aos/common/time.h"

namespace aos {
namespace logging {
namespace linux_code {
namespace {

void CheckTypeWritten(uint32_t type_id, LogFileWriter *writer,
                      ::std::unordered_set<uint32_t> *written_type_ids) {
  if (written_type_ids->count(type_id) > 0) return;
  if (MessageType::IsPrimitive(type_id)) return;

  const MessageType &type = type_cache::Get(type_id);
  for (int i = 0; i < type.number_fields; ++i) {
    CheckTypeWritten(type.fields[i]->type, writer, written_type_ids);
  }

  char buffer[1024];
  ssize_t size = type.Serialize(buffer, sizeof(buffer));
  if (size == -1) {
    LOG(WARNING, "%zu-byte buffer is too small to serialize type %s\n",
        sizeof(buffer), type.name.c_str());
    return;
  }
  LogFileMessageHeader *const output =
      writer->GetWritePosition(sizeof(LogFileMessageHeader) + size);

  output->time_sec = output->time_nsec = 0;
  output->source = getpid();
  output->name_size = 0;
  output->sequence = 0;
  output->level = FATAL;

  memcpy(output + 1, buffer, size);
  output->message_size = size;

  output->type = LogFileMessageHeader::MessageType::kStructType;
  futex_set(&output->marker);

  written_type_ids->insert(type_id);
}

void AllocateLogName(char **filename, const char *directory) {
  int fileindex = 0;
  DIR *const d = opendir(directory);
  if (d == nullptr) {
    PDie("could not open directory %s", directory);
  }
  int index = 0;
  while (true) {
    errno = 0;
    struct dirent *const dir = readdir(d);
    if (dir == nullptr) {
      if (errno == 0) {
        break;
      } else {
        PLOG(FATAL, "readdir(%p) failed", d);
      }
    } else {
      if (sscanf(dir->d_name, "aos_log-%d", &index) == 1) {
        if (index >= fileindex) {
          fileindex = index + 1;
        }
      }
    }
  }
  closedir(d);

  char previous[512];
  ::std::string path = ::std::string(directory) + "/aos_log-current";
  ssize_t len = ::readlink(path.c_str(), previous, sizeof(previous));
  if (len != -1) {
    previous[len] = '\0';
  } else {
    previous[0] = '\0';
    LOG(INFO, "Could not find aos_log-current\n");
    printf("Could not find aos_log-current\n");
  }
  if (asprintf(filename, "%s/aos_log-%03d", directory, fileindex) == -1) {
    PDie("couldn't create final name");
  }
  LOG(INFO, "Created log file (aos_log-%d) in directory (%s). Previous file "
            "was (%s).\n",
      fileindex, directory, previous);
  printf("Created log file (aos_log-%d) in directory (%s). Previous file was "
         "(%s).\n",
         fileindex, directory, previous);
}

#ifdef AOS_ARCHITECTURE_arm_frc
bool FoundThumbDrive(const char *path) {
  FILE *mnt_fp = setmntent("/etc/mtab", "r");
  if (mnt_fp == nullptr) {
    Die("Could not open /etc/mtab");
  }

  bool found = false;
  struct mntent mntbuf;
  char buf[256];
  while (!found) {
    struct mntent *mount_list = getmntent_r(mnt_fp, &mntbuf, buf, sizeof(buf));
    if (mount_list == nullptr) {
      break;
    }
    if (strcmp(mount_list->mnt_dir, path) == 0) {
      found = true;
    }
  }
  endmntent(mnt_fp);
  return found;
}

bool FindDevice(char *device, size_t device_size) {
  char test_device[10];
  for (char i = 'a'; i < 'z'; ++i) {
    snprintf(test_device, sizeof(test_device), "/dev/sd%c", i);
    LOG(INFO, "Trying to access %s\n", test_device);
    if (access(test_device, F_OK) != -1) {
      snprintf(device, device_size, "sd%c", i);
      return true;
    }
  }
  return false;
}
#endif

int BinaryLogReaderMain() {
  InitNRT();

#ifdef AOS_ARCHITECTURE_arm_frc
  char folder[128];

  {
    char dev_name[8];
    while (!FindDevice(dev_name, sizeof(dev_name))) {
      LOG(INFO, "Waiting for a device\n");
      printf("Waiting for a device\n");
      sleep(5);
    }
    snprintf(folder, sizeof(folder), "/media/%s1", dev_name);
    while (!FoundThumbDrive(folder)) {
      LOG(INFO, "Waiting for %s\n", folder);
      printf("Waiting for %s\n", folder);
      sleep(1);
    }
    snprintf(folder, sizeof(folder), "/media/%s1/", dev_name);
  }

  if (access(folder, F_OK) == -1) {
#else
  const char *folder = configuration::GetLoggingDirectory();
  if (access(folder, R_OK | W_OK) == -1) {
#endif
    LOG(FATAL, "folder '%s' does not exist. please create it\n", folder);
  }
  LOG(INFO, "logging to folder '%s'\n", folder);

  char *tmp;
  AllocateLogName(&tmp, folder);
  char *tmp2;
  if (asprintf(&tmp2, "%s/aos_log-current", folder) == -1) {
    PLOG(WARNING, "couldn't create current symlink name");
  } else {
    if (unlink(tmp2) == -1 && (errno != EROFS && errno != ENOENT)) {
      LOG(WARNING, "unlink('%s') failed", tmp2);
    }
    if (symlink(tmp, tmp2) == -1) {
      PLOG(WARNING, "symlink('%s', '%s') failed", tmp, tmp2);
    }
    free(tmp2);
  }
  int fd = open(tmp, O_SYNC | O_APPEND | O_RDWR | O_CREAT,
                S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
  free(tmp);
  if (fd == -1) {
    PLOG(FATAL, "opening file '%s' failed", tmp);
  }
  LogFileWriter writer(fd);

  RawQueue *queue = GetLoggingQueue();

  ::std::unordered_set<uint32_t> written_type_ids;
  off_t clear_type_ids_cookie = 0;

  while (true) {
    const LogMessage *const msg =
        static_cast<const LogMessage *>(queue->ReadMessage(RawQueue::kNonBlock));
    if (msg == NULL) {
      // If we've emptied the queue, then wait for a bit before starting to read
      // again so the queue can buffer up some logs. This avoids lots of context
      // switches and mutex contention which happens if we're constantly reading
      // new messages as they come in.
      static constexpr auto kSleepTime = ::aos::time::Time::InSeconds(0.1);
      ::aos::time::SleepFor(kSleepTime);
      continue;
    }

    const size_t raw_output_length =
        sizeof(LogFileMessageHeader) + msg->name_length + msg->message_length;
    size_t output_length = raw_output_length;
    if (msg->type == LogMessage::Type::kStruct) {
      output_length += sizeof(msg->structure.type_id) + sizeof(uint32_t) +
                       msg->structure.string_length;
      if (writer.ShouldClearSeekableData(&clear_type_ids_cookie,
                                         output_length)) {
        writer.ForceNewPage();
        written_type_ids.clear();
      }
      CheckTypeWritten(msg->structure.type_id, &writer, &written_type_ids);
    } else if (msg->type == LogMessage::Type::kMatrix) {
      output_length +=
          sizeof(msg->matrix.type) + sizeof(uint32_t) + sizeof(uint16_t) +
          sizeof(uint16_t) + msg->matrix.string_length;
      CHECK(MessageType::IsPrimitive(msg->matrix.type));
    }
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
      case LogMessage::Type::kStruct: {
        char *position = output_strings + msg->name_length;

        memcpy(position, &msg->structure.type_id,
               sizeof(msg->structure.type_id));
        position += sizeof(msg->structure.type_id);
        output->message_size += sizeof(msg->structure.type_id);

        const uint32_t length = msg->structure.string_length;
        memcpy(position, &length, sizeof(length));
        position += sizeof(length);
        memcpy(position, msg->structure.serialized,
               length + msg->message_length);
        position += length + msg->message_length;
        output->message_size += sizeof(length) + length;

        output->type = LogFileMessageHeader::MessageType::kStruct;
      } break;
      case LogMessage::Type::kMatrix: {
        char *position = output_strings + msg->name_length;

        memcpy(position, &msg->matrix.type, sizeof(msg->matrix.type));
        position += sizeof(msg->matrix.type);
        output->message_size += sizeof(msg->matrix.type);

        uint32_t length = msg->matrix.string_length;
        memcpy(position, &length, sizeof(length));
        position += sizeof(length);
        output->message_size += sizeof(length);

        uint16_t rows = msg->matrix.rows, cols = msg->matrix.cols;
        memcpy(position, &rows, sizeof(rows));
        position += sizeof(rows);
        memcpy(position, &cols, sizeof(cols));
        position += sizeof(cols);
        output->message_size += sizeof(rows) + sizeof(cols);
        CHECK_EQ(msg->message_length,
                 MessageType::Sizeof(msg->matrix.type) * rows * cols);

        memcpy(position, msg->matrix.data, msg->message_length + length);
        output->message_size += length;

        output->type = LogFileMessageHeader::MessageType::kMatrix;
      } break;
    }

    if (output->message_size - msg->message_length !=
        output_length - raw_output_length) {
      LOG(FATAL, "%zu != %zu\n", output->message_size - msg->message_length,
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
