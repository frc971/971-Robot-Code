#include <libgen.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <string>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/logging_impl.h"

extern "C" {
#include "stm32flash/parsers/parser.h"
#include "stm32flash/parsers/hex.h"
#include "stm32flash/serial.h"
#include "stm32flash/stm32.h"
#include "stm32flash/init.h"
}

int main(int argc, char **argv) {
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));

  if (argc < 2) {
    fputs("Need an argument saying which target to download.\n", stderr);
    return 1;
  }
  ::std::string target = argv[1];

  ::std::string device = "/dev/ttyUSB0";
  serial_baud_t baud_rate = SERIAL_BAUD_57600;

  ::std::string filename =
      ::std::string(dirname(strdup(argv[0]))) +
      "/../../../bbb_cape/src/cape/.obj/" + target + ".hex";

  int file = open(filename.c_str(), O_RDONLY);
  if (file == -1) {
    filename = target;
    file = open(filename.c_str(), O_RDONLY);
    if (file == -1) {
      LOG(FATAL, "open(%s, O_RDONLY) failed with %d: %s\n",
          filename.c_str(), errno, strerror(errno));
    } else {
      LOG(INFO, "using filename %s from the command line\n", filename.c_str());
    }
  }

  uint16_t start_address = 0;
  {
    uint8_t buffer[1 /* : */ + 2 /* record size */ + 4 /* address */ +
        2 /* record type */];
    ssize_t bytes = read(file, buffer, sizeof(buffer));
    if (close(file) == -1) {
      LOG(FATAL, "close(%d) failed with %d: %s\n",
          file, errno, strerror(errno));
    }
    if (bytes != sizeof(buffer)) {
      LOG(FATAL, "read %zd bytes from %s instead of %zu\n",
          bytes, filename.c_str(), sizeof(buffer));
    }
    if (buffer[0] != ':' || buffer[7] != '0' || buffer[8] != '0') {
      LOG(FATAL, "%s isn't a valid hex file that we know how to handle\n",
          filename.c_str());
    }
    for (int i = 0; i < 4; ++i) {
      uint8_t digit = buffer[3 + i];
      int value;
      if (digit >= '0' && digit <= '9') {
        value = digit - '0';
      } else if (digit >= 'a' && digit <= 'f') {
        value = digit - 'a';
      } else if (digit >= 'A' && digit <= 'F') {
        value = digit - 'A';
      } else {
        LOG(FATAL, "unknown hex digit %c\n", digit);
      }
      start_address |= value << (12 - (4 * i));
    }
    LOG(INFO, "start address = 0x%x\n", start_address);
  }

  parser_t *parser = &PARSER_HEX;
  void *p_st = parser->init();
  if (p_st == NULL) {
    LOG(FATAL, "%s parser failed to initialize.\n", parser->name);
  }
  if (parser->open(p_st, filename.c_str(), 0) != PARSER_ERR_OK) {
    LOG(FATAL, "opening file %s failed\n", filename.c_str());
  }

  serial_t *serial = serial_open(device.c_str());
  if (serial == NULL) {
    LOG(FATAL, "failed to open serial port %s because of %d: %s\n",
        device.c_str(), errno, strerror(errno));
  }
  if (serial_setup(serial, baud_rate,
                   SERIAL_BITS_8,
                   SERIAL_PARITY_EVEN,
                   SERIAL_STOPBIT_1) != SERIAL_ERR_OK) {
    LOG(FATAL, "setting up serial port %s failed because of %d: %s\n",
        device.c_str(), errno, strerror(errno));
  }
  LOG(INFO, "serial configuration: %s\n", serial_get_setup_str(serial));

  if (init_bl_entry(serial, NULL /* GPIO sequence */) == 0) {
    LOG(FATAL, "init_bl_entry(%p, NULL) failed\n", serial);
  }
  stm32_t *stm = stm32_init(serial, true);
  if (stm == NULL) {
    LOG(FATAL, "stm32_init(%p, true) failed\n", serial);
  }

  unsigned int last_byte = parser->size(p_st);
  unsigned int size = last_byte - start_address;

  // An array of the sizes of each sector.
  static const uint32_t kSectorSizes[12] = {0x4000, 0x4000, 0x4000, 0x4000,
                                            0x10000, 0x20000, 0x20000, 0x20000,
                                            0x20000, 0x20000, 0x20000, 0x20000};
  static const int kNumSectors = sizeof(kSectorSizes) / sizeof(kSectorSizes[0]);
  // The sector number that we're going to start writing at.
  int start_sector = 0;
  for (uint32_t address = 0; start_sector <= kNumSectors;
       address += kSectorSizes[start_sector++]) {
    if (start_sector == kNumSectors) {
      LOG(FATAL, "start address %x is too big\n", start_address);
    }
    if (address > start_address) {
      LOG(FATAL, "start address %x is not on a sector boundary\n",
          start_address);
    }
    if (address == start_address) break;
  }

  // The first sector that we're not going to erase.
  int end_sector = 0;
  for (uint32_t address = 0; end_sector <= kNumSectors;
       address += kSectorSizes[end_sector++]) {
    if (address > start_address + size) break;
    if (end_sector == kNumSectors) {
      LOG(FATAL, "%x bytes beyond start address of %x is too big\n",
          size, start_address);
    }
  }

  if (!stm32_erase_memory(stm, start_sector, end_sector - start_sector)) {
    LOG(FATAL, "failed to erase memory\n");
  }

  // Read all of the 0xFFs that the parser inserts to pad the data out.
  {
    uint8_t garbage[1024];
    uint32_t length = start_address;
    while (length > 0) {
      uint32_t read = ::std::min(sizeof(garbage), length);
      if (parser->read(p_st, garbage, &read) != PARSER_ERR_OK) {
        LOG(FATAL, "reading 0xFFs from the hex parser failed\n");
      }
      length -= read;
    }
  }

  uint32_t kFlashStart = 0x08000000;

  uint8_t buffer[256];  // 256 is the biggest size supported
  uint32_t completed = 0;
  while (completed < size) {
    uint32_t address = start_address + completed + kFlashStart;
    uint32_t length = ::std::min(size - completed, sizeof(buffer));
    if (parser->read(p_st, buffer, &length) != PARSER_ERR_OK) {
      LOG(FATAL, "reading file failed\n");
    }
    if (length == 0) {
      LOG(FATAL, "failed to read input file\n");
    }
    if ((length % 4) != 0) {
      // Pad the size we want to write to a multiple of 4 bytes.
      for (unsigned int i = 0; i < (4 - (length % 4)); ++i) {
        buffer[length++] = 0xFF;
      }
    }
    if (!stm32_write_memory(stm, address, buffer, length)) {
      LOG(FATAL, "failed to write memory at address 0x%x\n", address);
    }
    uint8_t compare_buffer[sizeof(buffer)];
    if (!stm32_read_memory(stm, address, compare_buffer, length)) {
      LOG(FATAL, "failed to read memory at address 0x%x\n", address);
    }
    if (memcmp(buffer, compare_buffer, length) != 0) {
      printf("\n");
      for (size_t i = 0; i < length; ++i) {
        LOG(DEBUG, "want %x have %x\n", buffer[i], compare_buffer[i]);
      }
      LOG(FATAL, "verify from 0x%x to 0x%x failed\n",
          address, address + length);
    }
    completed += length;
    printf("\rWrote and verified 0x%08x/0x%08x",
           completed, size);
    fflush(stdout);
  }
  printf("\n");

  if (init_bl_exit(stm, serial, NULL /* GPIO sequence */)) {
    LOG(INFO, "all done\n");
  } else {
    LOG(FATAL, "init_bl_exit failed\n");
  }
}
