#include "shared_mem.h"

#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>

// the path for the shared memory segment. see shm_open(3) for restrictions
#define AOS_SHM_NAME "/aos_shared_mem"
// Size of the shared mem segment.
// set to the maximum number that worked
#define SIZEOFSHMSEG (4096 * 27813)

ptrdiff_t aos_core_get_mem_usage(void) {
  return global_core->size -
      ((ptrdiff_t)global_core->mem_struct->msg_alloc -
       (ptrdiff_t)global_core->mem_struct);
}

struct aos_core global_core_data;
struct aos_core *global_core = NULL;

int aos_core_create_shared_mem(enum aos_core_create to_create) {
  global_core = &global_core_data;
  int shm;
before:
  if (to_create == create) {
    printf("shared_mem: creating\n");
    shm = shm_open(AOS_SHM_NAME, O_RDWR | O_CREAT | O_EXCL, 0666);
    global_core->owner = 1;
    if (shm == -1 && errno == EEXIST) {
      printf("shared_mem: going to shm_unlink(" AOS_SHM_NAME ")\n");
      if (shm_unlink(AOS_SHM_NAME) == -1) {
        fprintf(stderr, "shared_mem: shm_unlink(" AOS_SHM_NAME ") failed with of %d: %s\n", errno, strerror(errno));
      } else {
        goto before;
      }
    }
  } else {
    printf("shared_mem: not creating\n");
    shm = shm_open(AOS_SHM_NAME, O_RDWR, 0);
    global_core->owner = 0;
  }
  if (shm == -1) {
    fprintf(stderr, "shared_mem:"
            " shm_open(" AOS_SHM_NAME ", O_RDWR [| O_CREAT | O_EXCL, 0|0666)"
            " failed with %d: %s\n", errno, strerror(errno));
    return -1;
  }
  if (global_core->owner) {
    if (ftruncate(shm, SIZEOFSHMSEG) == -1) {
      fprintf(stderr, "shared_mem: fruncate(%d, 0x%zx) failed with %d: %s\n",
        shm, (size_t)SIZEOFSHMSEG, errno, strerror(errno));
      return -1;
    }
  }
  void *shm_address = mmap(
      (void *)SHM_START, SIZEOFSHMSEG, PROT_READ | PROT_WRITE,
      MAP_SHARED | MAP_FIXED | MAP_LOCKED | MAP_POPULATE, shm, 0);
  if (shm_address == MAP_FAILED) {
    fprintf(stderr, "shared_mem: mmap(%p, 0x%zx, stuff, stuff, %d, 0) failed"
            " with %d: %s\n",
            (void *)SHM_START, SIZEOFSHMSEG, shm, errno, strerror(errno));
    return -1;
  }
  printf("shared_mem: shm at: %p\n", shm_address);
  if (close(shm) == -1) {
    printf("shared_mem: close(%d(=shm) failed with %d: %s\n",
        shm, errno, strerror(errno));
  }
  if (shm_address != (void *)SHM_START) {
    fprintf(stderr, "shared_mem: shm isn't at hard-coded %p. at %p instead\n",
        (void *)SHM_START, shm_address);
    return -1;
  }
  return aos_core_use_address_as_shared_mem(shm_address, SIZEOFSHMSEG);
}

int aos_core_use_address_as_shared_mem(void *address, size_t size) {
  global_core->mem_struct = address;
  global_core->size = size;
  global_core->shared_mem = (uint8_t *)address + sizeof(*global_core->mem_struct);
  if (global_core->owner) {
    global_core->mem_struct->msg_alloc = (uint8_t *)address + global_core->size;
    init_shared_mem_core(global_core->mem_struct);
  }
  if (global_core->owner) {
    futex_set(&global_core->mem_struct->creation_condition);
  } else {
    if (futex_wait(&global_core->mem_struct->creation_condition) != 0) {
      fprintf(stderr, "waiting on creation_condition failed\n");
      return -1;
    }
  }
  fprintf(stderr, "shared_mem: end of create_shared_mem owner=%d\n",
          global_core->owner);
  return 0;
}

int aos_core_free_shared_mem(){
  void *shm_address = global_core->shared_mem;
      if (munmap((void *)SHM_START, SIZEOFSHMSEG) == -1) {
          fprintf(stderr, "shared_mem: munmap(%p, 0x%zx) failed with %d: %s\n",
        shm_address, SIZEOFSHMSEG, errno, strerror(errno));
          return -1;
      }
  if (global_core->owner) {
        if (shm_unlink(AOS_SHM_NAME)) {
            fprintf(stderr, "shared_mem: shm_unlink(" AOS_SHM_NAME ") failed with %d: %s\n",
          errno, strerror(errno));
            return -1;
        }
  }
  return 0;
}
