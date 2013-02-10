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

#include "aos/aos_core.h"
#include "aos/atom_code/core/LogFileCommon.h"

static const char *const kCRIOName = "CRIO";

int main() {
  aos::InitNRT();

  char *folder_tmp;
  if (asprintf(&folder_tmp, "%s/tmp/robot_logs", getpwuid(getuid())->pw_dir) == -1) {
    LOG(ERROR, "couldn't figure out what folder to use because of %d (%s)\n",
        errno, strerror(errno));
    return EXIT_FAILURE;
  }
  std::string hack("/home/driver/tmp/robot_logs"); // TODO(brians) remove this hack
  const char *folder = hack.c_str();
  if (access(folder, R_OK | W_OK) == -1) {
    fprintf(stderr,
            "LogReader: error: folder '%s' does not exist. please create it\n",
            folder);
    return EXIT_FAILURE;
  }
  LOG(INFO, "logging to folder '%s'\n", folder);

  const time_t t = time(NULL);
  char *tmp;
  if (asprintf(&tmp, "%s/aos_log-%jd", folder, static_cast<uintmax_t>(t)) == -1) {
    fprintf(stderr, "BinaryLogReader: couldn't create final name because of %d (%s)."
            " exiting\n", errno, strerror(errno));
    return EXIT_FAILURE;
  }
  char *tmp2;
  if (asprintf(&tmp2, "%s/aos_log-current", folder) == -1) {
    fprintf(stderr, "BinaryLogReader: couldn't create symlink name because of %d (%s)."
            " not creating current symlink\n", errno, strerror(errno));
  } else {
    if (unlink(tmp2) == -1 && (errno != EROFS && errno != ENOENT)) {
      fprintf(stderr, "BinaryLogReader: warning: unlink('%s') failed because of %d (%s)\n",
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
    fprintf(stderr, "BinaryLogReader: couldn't open file '%s' because of %d (%s)."
            " exiting\n", tmp, errno, strerror(errno));
    return EXIT_FAILURE;
  }
  aos::LogFileAccessor writer(fd, true);

  struct timespec timespec;
  time_t last_sec = 0;
  while (true) {
    clock_gettime(CLOCK_MONOTONIC, &timespec);
    if (last_sec != timespec.tv_sec) {
      LOG(INFO, "msyncing output\n");
      last_sec = timespec.tv_sec;
      writer.Sync();
    }

    const log_message *const msg = log_read_next();
    if (msg == NULL) continue;
    const log_crio_message *const crio_msg = reinterpret_cast<const log_crio_message *>(
        msg);

    size_t name_size, message_size;
    if (msg->source == -1) {
      name_size = strlen(kCRIOName);
      message_size = strlen(crio_msg->message);
    } else {
      name_size = strlen(msg->name);
      message_size = strlen(msg->message);
    }
    // add on size for terminating '\0'
    name_size += 1;
    message_size += 1;

    aos::LogFileMessageHeader *const output = writer.GetWritePosition(
        sizeof(aos::LogFileMessageHeader) + name_size + message_size);
    char *output_strings = reinterpret_cast<char *>(output) + sizeof(*output);
    output->name_size = name_size;
    output->message_size = message_size;
    output->source = msg->source;
    if (msg->source == -1) {
      output->level = crio_msg->level;
      // TODO(brians) figure out what time to put in
      output->sequence = crio_msg->sequence;
      memcpy(output_strings, kCRIOName, name_size);
      memcpy(output_strings + name_size, crio_msg->message, message_size);
    } else {
      output->level = msg->level;
      output->time_sec = msg->time.tv_sec;
      output->time_nsec = msg->time.tv_nsec;
      output->sequence = msg->sequence;
      memcpy(output_strings, msg->name, name_size);
      memcpy(output_strings + name_size, msg->message, message_size);
    }
    condition_set(&output->marker);

    log_free_message(msg);
  }

  aos::Cleanup();
}
