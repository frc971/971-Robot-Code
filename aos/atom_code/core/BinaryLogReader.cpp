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

#include <map>

#include "aos/atom_code/logging/atom_logging.h"
#include "aos/atom_code/core/LogFileCommon.h"
#include "aos/atom_code/init.h"
#include "aos/atom_code/configuration.h"

namespace aos {
namespace logging {
namespace atom {
namespace {

int BinaryLogReaderMain() {
  InitNRT();

  const char *folder = configuration::GetLoggingDirectory();
  if (access(folder, R_OK | W_OK) == -1) {
    LOG(FATAL, "folder '%s' does not exist. please create it\n", folder);
  }
  LOG(INFO, "logging to folder '%s'\n", folder);

  const time_t t = time(NULL);
  char *tmp;
  if (asprintf(&tmp, "%s/aos_log-%jd", folder, static_cast<uintmax_t>(t)) ==
      -1) {
    fprintf(stderr,
            "BinaryLogReader: couldn't create final name because of %d (%s)."
            " exiting\n", errno, strerror(errno));
    return EXIT_FAILURE;
  }
  char *tmp2;
  if (asprintf(&tmp2, "%s/aos_log-current", folder) == -1) {
    fprintf(stderr,
            "BinaryLogReader: couldn't create symlink name because of %d (%s)."
            " not creating current symlink\n", errno, strerror(errno));
  } else {
    if (unlink(tmp2) == -1 && (errno != EROFS && errno != ENOENT)) {
      fprintf(stderr,
              "BinaryLogReader: warning: unlink('%s') failed"
              " because of %d (%s)\n",
              tmp2, errno, strerror(errno));
    }
    if (symlink(tmp, tmp2) == -1) {
      fprintf(stderr, "BinaryLogReader: warning: symlink('%s', '%s') failed"
              " because of %d (%s)\n", tmp, tmp2, errno, strerror(errno));
    }
    free(tmp2);
  }
  int fd = open(tmp, O_SYNC | O_APPEND | O_RDWR | O_CREAT,
                S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
  free(tmp);
  if (fd == -1) {
    fprintf(stderr,
            "BinaryLogReader: couldn't open file '%s' because of %d (%s)."
            " exiting\n", tmp, errno, strerror(errno));
    return EXIT_FAILURE;
  }
  LogFileAccessor writer(fd, true);

  struct timespec timespec;
  time_t last_sec = 0;
  while (true) {
    clock_gettime(CLOCK_MONOTONIC, &timespec);
    if (last_sec != timespec.tv_sec) {
      LOG(INFO, "msyncing output\n");
      last_sec = timespec.tv_sec;
      writer.Sync();
    }

    const LogMessage *const msg = ReadNext();
    if (msg == NULL) continue;

    // add 1 for terminating '\0'
    size_t name_size = strlen(msg->name) + 1;
    size_t message_size = strlen(msg->message) + 1;

    LogFileMessageHeader *const output = writer.GetWritePosition(
        sizeof(LogFileMessageHeader) + name_size + message_size);
    char *output_strings = reinterpret_cast<char *>(output) + sizeof(*output);
    output->name_size = name_size;
    output->message_size = message_size;
    output->source = msg->source;
    output->level = msg->level;
    output->time_sec = msg->seconds;
    output->time_nsec = msg->nseconds;
    output->sequence = msg->sequence;
    memcpy(output_strings, msg->name, name_size);
    memcpy(output_strings + name_size, msg->message, message_size);
    futex_set(&output->marker);

    logging::atom::Free(msg);
  }

  Cleanup();
  return 0;
}

}  // namespace
}  // namespace atom
}  // namespace logging
}  // namespace aos

int main() {
  return ::aos::logging::atom::BinaryLogReaderMain();
}
