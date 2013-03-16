#include "DriverStationDisplay.h"

#include <stdarg.h>
#include <stdio.h>

#include "aos/common/logging/logging.h"

using namespace aos;

static const aos_type_sig signature = {sizeof(DriverStationDisplay), 1234, 10};
aos_queue *DriverStationDisplay::queue = NULL;
void DriverStationDisplay::GetQueue() {
  if (queue == NULL) {
    queue = aos_fetch_queue("DriverStationDisplay", &signature);
  }
}

void DriverStationDisplay::Send(int line, const char *fmt, ...) {
  GetQueue();
  DriverStationDisplay *msg = static_cast<DriverStationDisplay *>(
      aos_queue_get_msg(queue));
  if (msg == NULL) {
    LOG(WARNING, "could not get message to send '%s' to the DS queue\n", fmt);
    return;
  }
  msg->line = static_cast<uint8_t>(line);

  va_list ap;
  va_start(ap, fmt);
  int ret = vsnprintf(msg->data, sizeof(msg->data), fmt, ap);
  va_end(ap);
  if (ret < 0) {
    LOG(WARNING, "could not format '%s' with arguments\n", fmt);
    aos_queue_free_msg(queue, msg);
    return;
  } else if (static_cast<uintmax_t>(ret) >=
             static_cast<uintmax_t>(sizeof(msg->data))) {
    LOG(WARNING, "format '%s' ended up longer than the max size (%zd)\n",
        fmt, sizeof(msg->data));
  }
  
  if (aos_queue_write_msg(queue, msg, NON_BLOCK) < 0) {
    LOG(ERROR, "writing '%s' (line %hhd) failed\n", msg->data, msg->line);
    aos_queue_free_msg(queue, msg);
  }
}

const DriverStationDisplay *DriverStationDisplay::GetNext() {
  GetQueue();
  return static_cast<const DriverStationDisplay *>(aos_queue_read_msg(queue, NON_BLOCK));
}
void DriverStationDisplay::Free(const DriverStationDisplay *msg) {
  GetQueue();
  aos_queue_free_msg(queue, msg);
}

