#ifndef CAPE_BOOTLOADER_HANDOFF_H_
#define CAPE_BOOTLOADER_HANDOFF_H_

// This file has constants and functions for dealing with the handoff between
// the bootloader and the main code.

// How much flash the bootloader has (starting at address 0).
#define BOOTLOADER_FLASH_SIZE 0x8000
// Where the main code's flash starts.
#define MAIN_FLASH_START BOOTLOADER_FLASH_SIZE

#define RAM_START 0x20000000
#define RAM_SIZE 0x20000

#endif  // CAPE_BOOTLOADER_HANDOFF_H_
