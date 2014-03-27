#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <inttypes.h>
#include <errno.h>

#include <algorithm>

#include "aos/linux_code/logging/binary_log_file.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/common/queue_types.h"

using ::aos::logging::linux_code::LogFileMessageHeader;

namespace {

const char *kArgsHelp = "[OPTION]... [FILE]\n"
    "Display log file FILE (created by BinaryLogReader) to stdout.\n"
    "FILE is \"aos_log-current\" by default.\n"
    "\n"
    "  -n, --name NAME       only display entries from processes named NAME\n"
    "  -l, --level LEVEL     "
      "only display log entries at least as important as LEVEL\n"
    "  // -p, --pid PID        only display log entries from process PID\n"
    "  -f, --follow          "
      "wait when the end of the file is reached (implies --end)\n"
    "  -t, --terminate       stop when the end of file is reached (default)\n"
    "  -b, --beginning       start at the beginning of the file (default)\n"
    "  -e, --end             start at the end of the file\n"
    "  -s, --skip NUMBER     skip NUMBER matching logs\n"
    "  // -m, --max NUMBER     only display up to NUMBER logs\n"
    "  // -o, --format FORMAT  use FORMAT to display log entries\n"
    "  -h, --help            display this help and exit\n"
    "\n"
    "LEVEL must be DEBUG, INFO, WARNING, ERROR, or FATAL.\n"
    "  It defaults to INFO.\n"
    "\n"
    "TODO(brians) implement the commented out ones and changing FILE\n";

void PrintHelpAndExit() {
  fprintf(stderr, "Usage: %s %s", program_invocation_name, kArgsHelp);

  exit(EXIT_SUCCESS);
}

}  // namespace

int main(int argc, char **argv) {
  const char *filter_name = NULL;
  size_t filter_length = 0;
  log_level filter_level = INFO;
  bool follow = false;
  // Whether we need to skip everything until we get to the end of the file.
  bool skip_to_end = false;
  const char *filename = "aos_log-current";

  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));

  while (true) {
    static struct option long_options[] = {
      {"name", required_argument, NULL, 'n'},
      {"level", required_argument, NULL, 'l'},
      {"pid", required_argument, NULL, 'p'},

      {"follow", no_argument, NULL, 'f'},
      {"terminate", no_argument, NULL, 't'},
      {"beginning", no_argument, NULL, 'b'},
      {"end", no_argument, NULL, 'e'},
      {"skip", required_argument, NULL, 's'},
      {"max", required_argument, NULL, 'm'},

      {"format", required_argument, NULL, 'o'},

      {"help", no_argument, NULL, 'h'},
      {0, 0, 0, 0}
    };
    int option_index = 0;

    const int c = getopt_long(argc, argv, "n:l:p:fts:m:o:h",
                    long_options, &option_index);
    if (c == -1) { // if we're at the end
      break;
    }
    switch (c) {
      case 0:
        fprintf(stderr, "LogDisplayer: got a 0 option but didn't set up any\n");
        abort();
      case 'n':
        filter_name = optarg;
        filter_length = strlen(filter_name);
        break;
      case 'l':
        filter_level = ::aos::logging::str_log(optarg);
        if (filter_level == LOG_UNKNOWN) {
          fprintf(stderr, "LogDisplayer: unknown log level '%s'\n", optarg);
          exit(EXIT_FAILURE);
        }
        break;
      case 'p':
        abort();
        break;
      case 'f':
        follow = true;
        skip_to_end = true;
        break;
      case 't':
        follow = false;
        break;
      case 'b':
        skip_to_end = false;
        break;
      case 'e':
        skip_to_end = true;
        break;
      case 'm':
        abort();
        break;
      case 'o':
        abort();
        break;
      case 'h':
        PrintHelpAndExit();
        break;
      case '?':
        break;
      default:
        fprintf(stderr, "LogDisplayer: in a bad spot (%s: %d)\n",
                __FILE__, __LINE__);
        abort();
    }
  }

  fprintf(stderr, "displaying down to level %s from file '%s'\n",
          ::aos::logging::log_str(filter_level), filename);
  if (optind < argc) {
    fprintf(stderr, "non-option ARGV-elements: ");
    while (optind < argc) {
      fprintf(stderr, "%s\n", argv[optind++]);
    }
  }

  int fd = open(filename, O_RDONLY);
  if (fd == -1) {
    fprintf(stderr, "error: couldn't open file '%s' for reading because of %s\n",
            filename, strerror(errno));
    exit(EXIT_FAILURE);
  }
  ::aos::logging::linux_code::LogFileAccessor accessor(fd, false);

  if (skip_to_end) {
    fputs("skipping old logs...\n", stderr);
  }

  const LogFileMessageHeader *msg;
  ::aos::logging::LogMessage log_message;
  do {
    msg = accessor.ReadNextMessage(follow);
    if (msg == NULL) {
      fputs("reached end of file\n", stderr);
      return 0;
    }

    if (msg->type == LogFileMessageHeader::MessageType::kStructType) {
      size_t bytes = msg->message_size;
      ::aos::MessageType *type = ::aos::MessageType::Deserialize(
          reinterpret_cast<const char *>(msg + 1), &bytes);
      ::aos::type_cache::Add(*type);
      continue;
    }

    if (skip_to_end) {
      if (accessor.IsLastPage()) {
        fputs("done skipping old logs\n", stderr);
        skip_to_end = false;
      } else {
        continue;
      }
    }

    if (::aos::logging::log_gt_important(filter_level, msg->level)) continue;
    if (filter_name != NULL) {
      if (filter_length != msg->name_size) continue;
      if (memcmp(filter_name,
                 reinterpret_cast<const char *>(msg) + sizeof(*msg),
                 filter_length) !=
          0) {
        continue;
      }
    }

    log_message.source = msg->source;
    log_message.sequence = msg->sequence;
    log_message.level = msg->level;
    log_message.seconds = msg->time_sec;
    log_message.nseconds = msg->time_nsec;
    memcpy(log_message.name, reinterpret_cast<const char *>(msg) + sizeof(*msg),
           ::std::min<size_t>(sizeof(log_message.name), msg->name_size));
    log_message.message_length = msg->message_size;
    log_message.name_length = msg->name_size;

    switch (msg->type) {
      case LogFileMessageHeader::MessageType::kStruct: {
        const char *position =
            reinterpret_cast<const char *>(msg + 1) + msg->name_size;
        memcpy(&log_message.structure.type_id, position,
               sizeof(log_message.structure.type_id));
        position += sizeof(log_message.structure.type_id);

        uint32_t length;
        memcpy(&length, position, sizeof(length));
        log_message.structure.string_length = length;
        position += sizeof(length);
        memcpy(log_message.structure.serialized, position, length);
        position += length;

        log_message.message_length -=
            sizeof(log_message.structure.type_id) + sizeof(uint32_t) +
            log_message.structure.string_length;
        memcpy(log_message.structure.serialized +
                   log_message.structure.string_length,
               position, log_message.message_length);

        log_message.type = ::aos::logging::LogMessage::Type::kStruct;
        break;
      }
      case LogFileMessageHeader::MessageType::kMatrix: {
        const char *position =
            reinterpret_cast<const char *>(msg + 1) + msg->name_size;
        memcpy(&log_message.matrix.type, position,
               sizeof(log_message.matrix.type));
        position += sizeof(log_message.matrix.type);

        uint32_t length;
        memcpy(&length, position, sizeof(length));
        log_message.matrix.string_length = length;
        position += sizeof(length);

        uint16_t rows;
        memcpy(&rows, position, sizeof(rows));
        log_message.matrix.rows = rows;
        position += sizeof(rows);
        uint16_t cols;
        memcpy(&cols, position, sizeof(cols));
        log_message.matrix.cols = cols;
        position += sizeof(cols);

        log_message.message_length -=
            sizeof(log_message.matrix.type) + sizeof(uint32_t) +
            sizeof(uint16_t) + sizeof(uint16_t) + length;
        CHECK_EQ(
            log_message.message_length,
            ::aos::MessageType::Sizeof(log_message.matrix.type) * rows * cols);
        memcpy(log_message.matrix.data, position,
               log_message.message_length + length);

        log_message.type = ::aos::logging::LogMessage::Type::kMatrix;
        break;
      } case LogFileMessageHeader::MessageType::kString:
        memcpy(
            log_message.message,
            reinterpret_cast<const char *>(msg) + sizeof(*msg) + msg->name_size,
            ::std::min<size_t>(sizeof(log_message.message), msg->message_size));
        log_message.type = ::aos::logging::LogMessage::Type::kString;
        break;
      case LogFileMessageHeader::MessageType::kStructType:
        LOG(FATAL, "shouldn't get here\n");
        break;
    };
    ::aos::logging::internal::PrintMessage(stdout, log_message);
  } while (msg != NULL);
}
