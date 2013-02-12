#include "aos/common/die.h"

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#ifdef __VXWORKS__
#include <taskLib.h>
// Have to re-declare it with __attribute__((noreturn)).
extern "C" void abort() __attribute__((noreturn));
#endif

#include <string>

#include "aos/aos_stdint.h"

namespace aos {

void Die(const char *format, ...) {
  va_list args;
  va_start(args, format);
  VDie(format, args);
}

namespace {
// Calculates the filename to dump the message into.
const std::string GetFilename() {
#ifdef __VXWORKS__
  const char *name = taskName(0);  // get the name of this task
  if (name == NULL) name = "<unknown>";
  const std::string first_part = "/aos_fatal_error.";
  return first_part + std::string(name);
#else
  char *filename;
  if (asprintf(&filename, "/tmp/aos_fatal_error.%jd",
               static_cast<intmax_t>(getpid())) > 0) {
    std::string r(filename);
    free(filename);
    return r;
  } else {
    fprintf(stderr, "aos fatal: asprintf(%p, \"thingie with %%jd\", %jd)"
            " failed with %d (%s)\n", &filename,
            static_cast<intmax_t>(getpid()), errno, strerror(errno));
    return std::string();
  }
#endif
}
}  // namespace

void VDie(const char *format, va_list args_in) {
  va_list args;

  fputs("aos fatal: ERROR!! details following\n", stderr);
  va_copy(args, args_in);
  vfprintf(stderr, format, args);
  va_end(args);
  fputs("aos fatal: ERROR!! see stderr for details\n", stdout);

  const std::string filename = GetFilename();
  if (!filename.empty()) {
    FILE *error_file = fopen(filename.c_str(), "w");
    if (error_file != NULL) {
      va_copy(args, args_in);
      vfprintf(error_file, format, args);
      va_end(args);
      fclose(error_file);
    } else {
      fprintf(stderr, "aos fatal: fopen('%s', \"w\") failed with %d (%s)\n",
              filename.c_str(), errno, strerror(errno));
    }
  }

  abort();
}

}  // namespace aos
