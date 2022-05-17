#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "aos/ipc_lib/lockless_queue.h"

int main(int argc, char **argv) {
  CHECK_EQ(argc, 2);
  const char *path = argv[1];

  struct stat st;
  PCHECK(lstat(path, &st) == 0);
  CHECK_NE(st.st_size, 0);
  const size_t size = st.st_size;

  // File already exists.
  int fd = open(path, O_RDWR, O_CLOEXEC);
  PCHECK(fd != -1) << ": Failed to open " << path;

  void *data = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  PCHECK(data != MAP_FAILED);
  PCHECK(close(fd) == 0);

  aos::ipc_lib::PrintLocklessQueueMemory(
      reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(data));

  PCHECK(munmap(data, size) == 0);
}
