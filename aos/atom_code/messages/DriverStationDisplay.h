#ifndef AOS_ATOM_CODE_MESSAGES_DRIVER_STATION_DISPLAY_H_
#define AOS_ATOM_CODE_MESSAGES_DRIVER_STATION_DISPLAY_H_

#include <stdint.h>
#include <string.h>

#include "aos/common/type_traits.h"
#include "aos/atom_code/ipc_lib/queue.h"

namespace aos {
const size_t kLineLength = 21;

struct DriverStationDisplay {
  static void Send(int line, const char *fmt, ...)
      __attribute__((format(printf, 2, 3)));
  static const DriverStationDisplay *GetNext(); // returns NULL if there are no more
  static void Free(const DriverStationDisplay *msg);

  uint8_t line;
  char data[kLineLength + 1];

 private:
  static void GetQueue();
  static aos_queue *queue;
};
static_assert(shm_ok<DriverStationDisplay>::value,
              "DriverStationDisplay will go through shared memory");
} // namespace aos

#endif

