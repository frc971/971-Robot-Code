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
#include <inttypes.h>

#include "aos/aos_core.h"
#include "aos/atom_code/core/LogFileCommon.h"

static const char *const kCRIOName = "CRIO";

int main() {
  aos::InitNRT();

  const time_t t = time(NULL);
  printf("starting at %jd----------------------------------\n", static_cast<uintmax_t>(t));

  int index = 0;
  while (true) {
    const log_message *const msg = log_read_next2(BLOCK, &index);
    if (msg == NULL) continue;
    const log_crio_message *const crio_msg = reinterpret_cast<const log_crio_message *>(
        msg);

    if (msg->source == -1) {
      printf("CRIO(%03"PRId8"): %s at %f: %s", crio_msg->sequence,
             log_str(crio_msg->level), crio_msg->time, crio_msg->message);
    } else {
      printf("%s(%"PRId32")(%03"PRId8"): %s at %010jus%09luns: %s",
             msg->name, msg->source, msg->sequence, log_str(msg->level),
             static_cast<uintmax_t>(msg->time.tv_sec), msg->time.tv_nsec, msg->message);
    }

    log_free_message(msg);
  }

  aos::Cleanup();
}
