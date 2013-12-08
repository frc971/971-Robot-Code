#include <stdint.h>

#include "cape/bootloader_handoff.h"

// Sets everything up and then jumps to the main code.
static void jump_to_main(void) __attribute__((noreturn));
static void jump_to_main(void) {
  // 0x20008000
  __asm__ __volatile__(
      "mov sp, %[stack]\n\t"
      "bx %[reset]" : :
      [stack]"r"(RAM_START + RAM_SIZE), [reset]"r"(MAIN_FLASH_START | 1)
      : "memory");
  __builtin_unreachable();
}

void _start(void) {
  jump_to_main();
}
