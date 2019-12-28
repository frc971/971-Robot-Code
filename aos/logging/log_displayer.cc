#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <inttypes.h>

#include <algorithm>
#include <memory>
#include <string>

#include "aos/configuration.h"
#include "aos/logging/binary_log_file.h"
#include "aos/logging/logging.h"
#include "aos/logging/implementations.h"
#include "aos/logging/printf_formats.h"
#include "aos/util/string_to_num.h"

using ::aos::logging::linux_code::LogFileMessageHeader;

namespace {

const char *kArgsHelp = "[OPTION]... [FILE]\n"
    "Display log file FILE (created by BinaryLogReader) to stdout.\n"
    "FILE is \"aos_log-current\" by default.\n"
    "FILE can also be \"-\" to read from standard input.\n"
    "\n"
    "  -n, --name-prefix NAME  "
    "only display entries from processes which share NAME as a prefix\n"
    "  -N, --name NAME         only display entries from processes named NAME\n"
    "  -l, --level LEVEL       "
      "only display log entries at least as important as LEVEL\n"
    "  -p, --pid PID           only display log entries from process PID\n"
    "  -f, --follow            "
      "wait when the end of the file is reached (implies --end)\n"
    "  -t, --terminate         stop when the end of file is reached (default)\n"
    "  -b, --beginning         start at the beginning of the file (default)\n"
    "  -e, --end               start at the end of the file\n"
    "  -s, --skip NUMBER       skip NUMBER matching logs\n"
    "  -m, --max NUMBER        only display up to NUMBER logs\n"
    "  // -o, --format FORMAT  use FORMAT to display log entries\n"
    "  -h, --help              display this help and exit\n"
    "\n"
    "LEVEL must be DEBUG, INFO, WARNING, ERROR, or FATAL.\n"
    "  It defaults to INFO.\n"
    "\n"
    "TODO(brians) implement the commented out ones.\n";

const char *kExampleUsages = "To view logs from the shooter:\n"
    "\t`log_displayer -n shooter`\n"
    "To view debug logs from the shooter:\n"
    "\t`log_displayer -n shooter -l DEBUG`\n"
    "To view what the shooter is logging in realtime:\n"
    "\t`log_displayer -f -n shooter`\n"
    "To view shooter logs from an old log file:\n"
    "\t`log_displayer aos_log-<number> -n shooter`\n"
    "To view the statuses of the shooter hall effects in realtime:\n"
    "\t`log_displayer -f -n shooter -l DEBUG | grep .Position`\n";

std::string GetRootDirectory() {
  ssize_t size = 0;
  std::unique_ptr<char[]> retu;
  while (true) {
    size += 256;
    retu.reset(new char[size]);

    ssize_t ret = readlink("/proc/self/exe", retu.get(), size);
    if (ret < 0) {
      if (ret != -1) {
        LOG(WARNING) << "it returned " << ret << ", not -1";
      }

      PLOG(FATAL) << "readlink(\"/proc/self/exe\", " << retu.get() << ", " << size
                  << ") failed";
    }
    if (ret < size) {
      void *last_slash = memrchr(retu.get(), '/', ret);
      if (last_slash == NULL) {
        retu.get()[ret] = '\0';
        LOG(FATAL) << "couldn't find a '/' in \"" << retu.get() << "\"";
      }
      *static_cast<char *>(last_slash) = '\0';
      LOG(INFO) << "got a root dir of \"" << retu.get() << "\"";
      return std::string(retu.get());
    }
  }
}


void PrintHelpAndExit() {
  fprintf(stderr, "Usage: %s %s", program_invocation_name, kArgsHelp);
  fprintf(stderr, "\nExample usages:\n\n%s", kExampleUsages);

  // Get the possible executables from start_list.txt.
  FILE *start_list = fopen("start_list.txt", "r");
  if (!start_list) {
    ::std::string path(GetRootDirectory());
    path += "/start_list.txt";
    start_list = fopen(path.c_str(), "r");
    if (!start_list) {
      printf(
          "\nCannot open start_list.txt. This means that the\npossible "
          "arguments for the -n option cannot be shown. log_displayer\nlooks "
          "for start_list.txt in the current working directory and in\n%s.\n\n",
          path.c_str());
      AOS_PLOG(FATAL, "Unable to open start_list.txt");
    }
  }

  // Get file size.
  if (fseek(start_list, 0, SEEK_END)) {
    AOS_PLOG(FATAL, "fseek() failed while reading start_list.txt");
  }
  int size = ftell(start_list);
  if (size < 0) {
    AOS_PLOG(FATAL, "ftell() failed while reading start_list.txt");
  }
  rewind(start_list);

  ::std::unique_ptr<char[]> contents(new char[size + 1]);
  if (contents == NULL) {
    AOS_LOG(FATAL, "malloc() failed while reading start_list.txt.\n");
  }
  size_t bytes_read = fread(contents.get(), 1, size, start_list);
  if (bytes_read < static_cast<size_t>(size)) {
    AOS_LOG(FATAL, "Read %zu bytes from start_list.txt, expected %d.\n",
            bytes_read, size);
  }

  // printf doesn't like strings without the \0.
  contents[size] = '\0';
  fprintf(stderr, "\nPossible arguments for the -n option:\n%s", contents.get());

  if (fclose(start_list)) {
    AOS_LOG(FATAL, "fclose() failed.\n");
  }

  exit(EXIT_SUCCESS);
}

}  // namespace

int main(int argc, char **argv) {
  const char *filter_name = nullptr, *filter_exact_name = nullptr;
  size_t filter_length = 0;
  log_level filter_level = INFO;
  bool follow = false;
  // Whether we need to skip everything until we get to the end of the file.
  bool skip_to_end = false;
  const char *filename = "aos_log-current";
  int display_max = 0;
  int32_t source_pid = -1;

  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));

  while (true) {
    static struct option long_options[] = {
      {"name-prefix", required_argument, NULL, 'n'},
      {"name", required_argument, NULL, 'N'},
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

    const int c = getopt_long(argc, argv, "N:n:l:p:fts:m:o:h",
                    long_options, &option_index);
    if (c == -1) { // if we're at the end
      break;
    }
    switch (c) {
      case 0:
        fputs("LogDisplayer: got a 0 option but didn't set up any\n", stderr);
        abort();
      case 'n':
        filter_name = optarg;
        filter_exact_name = nullptr;
        filter_length = strlen(filter_name);
        break;
      case 'N':
        filter_exact_name = optarg;
        filter_name = nullptr;
        filter_length = strlen(filter_exact_name);
        break;
      case 'l':
        filter_level = ::aos::logging::str_log(optarg);
        if (filter_level == LOG_UNKNOWN) {
          fprintf(stderr, "LogDisplayer: unknown log level '%s'\n", optarg);
          exit(EXIT_FAILURE);
        }
        break;
      case 'p':
        if (!::aos::util::StringToNumber(::std::string(optarg), &source_pid)) {
          fprintf(stderr, "ERROR: -p expects a number, not '%s'.\n", optarg);
          exit(EXIT_FAILURE);
        }
        if (source_pid < 0) {
          fprintf(stderr, "LogDisplayer: invalid pid '%s'\n", optarg);
          exit(EXIT_FAILURE);
        }
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
        if (!::aos::util::StringToNumber(::std::string(optarg), &display_max)) {
          fprintf(stderr, "ERROR: -m expects a number, not '%s'.\n", optarg);
          exit(EXIT_FAILURE);
        }
        if (display_max <= 0) {
          fprintf(stderr, "LogDisplayer: invalid max log number '%s'\n",
              optarg);
          exit(EXIT_FAILURE);
        }
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

  if (optind < argc) {
    // We got a filename.
    filename = argv[optind++];
  }
  if (optind < argc) {
    fputs("non-option ARGV-elements: ", stderr);
    while (optind < argc) {
      fprintf(stderr, "%s\n", argv[optind++]);
    }
  }

  int fd;
  if (strcmp(filename, "-") == 0) {
    if (skip_to_end) {
      fputs("Can't skip to end of stdin!\n", stderr);
      return EXIT_FAILURE;
    }
    fd = STDIN_FILENO;
  } else {
    fd = open(filename, O_RDONLY);
  }

  fprintf(stderr, "displaying down to level %s from file '%s'\n",
      ::aos::logging::log_str(filter_level), filename);

  if (fd == -1) {
    AOS_PLOG(FATAL, "couldn't open file '%s' for reading", filename);
  }
  ::aos::logging::linux_code::LogFileReader reader(fd);

  if (skip_to_end) {
    fputs("skipping old logs...\n", stderr);
    reader.SkipToLastSeekablePage();
  }

  const LogFileMessageHeader *msg;
  int displayed = 0;
  do {
    msg = reader.ReadNextMessage(follow);
    if (msg == NULL) {
      fputs("reached end of file\n", stderr);
      return 0;
    }

    if (source_pid >= 0 && msg->source != source_pid) {
      // Message is from the wrong process.
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

    const char *position =
        reinterpret_cast<const char *>(msg + 1);

    if (filter_name != nullptr) {
      const size_t compare_length =
          ::std::min<size_t>(filter_length, msg->name_size);
      if (memcmp(filter_name, position, compare_length) != 0) {
        continue;
      }
      if (compare_length < msg->name_size) {
        if (position[compare_length] != '.') continue;
      }
    }

    if (filter_exact_name != nullptr) {
      if (filter_length != msg->name_size) continue;
      if (memcmp(filter_exact_name, position, filter_length) != 0) {
        continue;
      }
    }

    if (display_max && displayed++ >= display_max) {
      fputs("Not displaying the rest of the messages.\n", stderr);
      return 0;
    }

    position += msg->name_size;

#define BASE_ARGS                                                           \
  AOS_LOGGING_BASE_ARGS(                                                    \
      msg->name_size, reinterpret_cast<const char *>(msg + 1), msg->source, \
      msg->sequence, msg->level, msg->time_sec, msg->time_nsec)
    switch (msg->type) {
      case LogFileMessageHeader::MessageType::kString:
        fprintf(stdout, AOS_LOGGING_BASE_FORMAT "%.*s", BASE_ARGS,
                static_cast<int>(msg->message_size), position);
        break;
      case LogFileMessageHeader::MessageType::kStruct:
      case LogFileMessageHeader::MessageType::kMatrix:
        AOS_LOG(FATAL, "Unsupported matrix or struct\n");
        break;
      case LogFileMessageHeader::MessageType::kStructType:
        AOS_LOG(FATAL, "shouldn't get here\n");
        break;
    }
#undef BASE_ARGS
  } while (msg != NULL);
}
