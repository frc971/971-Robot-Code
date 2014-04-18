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
#include "aos/common/queue_types.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/common/logging/logging_printf_formats.h"

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
    fprintf(stderr,
            "error: couldn't open file '%s' for reading because of %s\n",
            filename, strerror(errno));
    exit(EXIT_FAILURE);
  }
  ::aos::logging::linux_code::LogFileReader reader(fd);

  if (skip_to_end) {
    fputs("skipping old logs...\n", stderr);
  }

  const LogFileMessageHeader *msg;
  do {
    msg = reader.ReadNextMessage(follow);
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
      if (reader.IsLastPage()) {
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

    const char *position =
        reinterpret_cast<const char *>(msg + 1) + msg->name_size;
#define BASE_ARGS                                                           \
  AOS_LOGGING_BASE_ARGS(                                                    \
      msg->name_size, reinterpret_cast<const char *>(msg + 1), msg->source, \
      msg->sequence, msg->level, msg->time_sec, msg->time_nsec)
    switch (msg->type) {
      case LogFileMessageHeader::MessageType::kString:
        fprintf(stdout, AOS_LOGGING_BASE_FORMAT "%.*s", BASE_ARGS,
                static_cast<int>(msg->message_size), position);
        break;
      case LogFileMessageHeader::MessageType::kStruct: {
        uint32_t type_id;
        memcpy(&type_id, position, sizeof(type_id));
        position += sizeof(type_id);

        uint32_t string_length;
        memcpy(&string_length, position, sizeof(string_length));
        position += sizeof(string_length);

        char buffer[2048];
        size_t output_length = sizeof(buffer);
        size_t input_length =
            msg->message_size -
            (sizeof(type_id) + sizeof(uint32_t) + string_length);
        if (!PrintMessage(buffer, &output_length, position + string_length,
                          &input_length, ::aos::type_cache::Get(type_id))) {
          LOG(FATAL, "printing message (%.*s) of type %s into %zu-byte buffer "
                     "failed\n",
              static_cast<int>(string_length), position,
              ::aos::type_cache::Get(type_id).name.c_str(), sizeof(buffer));
        }
        if (input_length > 0) {
          LOG(WARNING, "%zu extra bytes on message of type %s\n",
              input_length, ::aos::type_cache::Get(type_id).name.c_str());
        }
        fprintf(stdout, AOS_LOGGING_BASE_FORMAT "%.*s: %.*s\n", BASE_ARGS,
                static_cast<int>(string_length), position,
                static_cast<int>(sizeof(buffer) - output_length), buffer);
      } break;
      case LogFileMessageHeader::MessageType::kMatrix: {
        uint32_t type;
        memcpy(&type, position, sizeof(type));
        position += sizeof(type);

        uint32_t string_length;
        memcpy(&string_length, position, sizeof(string_length));
        position += sizeof(string_length);

        uint16_t rows;
        memcpy(&rows, position, sizeof(rows));
        position += sizeof(rows);
        uint16_t cols;
        memcpy(&cols, position, sizeof(cols));
        position += sizeof(cols);

        const size_t matrix_bytes =
            msg->message_size -
            (sizeof(type) + sizeof(uint32_t) + sizeof(uint16_t) +
             sizeof(uint16_t) + string_length);
        CHECK_EQ(matrix_bytes, ::aos::MessageType::Sizeof(type) * rows * cols);
        char buffer[2048];
        size_t output_length = sizeof(buffer);
        if (!::aos::PrintMatrix(buffer, &output_length,
                                position + string_length, type, rows, cols)) {
          LOG(FATAL, "printing %dx%d matrix of type %" PRIu32 " failed\n", rows,
              cols, type);
        }
        fprintf(stdout, AOS_LOGGING_BASE_FORMAT "%.*s: %.*s\n", BASE_ARGS,
                static_cast<int>(string_length), position,
                static_cast<int>(sizeof(buffer) - output_length), buffer);
      } break;
      case LogFileMessageHeader::MessageType::kStructType:
        LOG(FATAL, "shouldn't get here\n");
        break;
    }
#undef BASE_ARGS
  } while (msg != NULL);
}
