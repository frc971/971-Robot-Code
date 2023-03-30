#include "frc971/can_logger/asc_logger.h"

#include <linux/can.h>

namespace frc971 {
namespace can_logger {

AscLogger::AscLogger(aos::EventLoop *event_loop, const std::string &filename)
    : output_(filename), event_loop_(event_loop) {
  CHECK(output_);
  event_loop->MakeWatcher(
      "/can", [this](const CanFrame &frame) { HandleFrame(frame); });
}

void AscLogger::HandleFrame(const CanFrame &frame) {
  if (!first_frame_monotonic_) {
    aos::monotonic_clock::time_point time(
        std::chrono::nanoseconds(frame.monotonic_timestamp_ns()));

    first_frame_monotonic_ = time;

    WriteHeader(output_, event_loop_->realtime_now());
  }

  WriteFrame(output_, frame);
}

void AscLogger::WriteHeader(std::ostream &file,
                            aos::realtime_clock::time_point start_time) {
  file << "date " << start_time << "\n";
  file << "base hex  timetamps absolute\n";
  file << "no internal events logged\n";
}

namespace {

static const unsigned char len2dlc[] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8, /* 0 - 8 */
    9,  9,  9,  9,                     /* 9 - 12 */
    10, 10, 10, 10,                    /* 13 - 16 */
    11, 11, 11, 11,                    /* 17 - 20 */
    12, 12, 12, 12,                    /* 21 - 24 */
    13, 13, 13, 13, 13, 13, 13, 13,    /* 25 - 32 */
    14, 14, 14, 14, 14, 14, 14, 14,    /* 33 - 40 */
    14, 14, 14, 14, 14, 14, 14, 14,    /* 41 - 48 */
    15, 15, 15, 15, 15, 15, 15, 15,    /* 49 - 56 */
    15, 15, 15, 15, 15, 15, 15, 15};   /* 57 - 64 */

/* map the sanitized data length to an appropriate data length code */
unsigned char can_fd_len2dlc(unsigned char len) {
  if (len > 64) return 0xF;

  return len2dlc[len];
}

#define ASC_F_RTR 0x00000010
#define ASC_F_FDF 0x00001000
#define ASC_F_BRS 0x00002000
#define ASC_F_ESI 0x00004000

}  // namespace

void AscLogger::WriteFrame(std::ostream &file, const CanFrame &frame) {
  aos::monotonic_clock::time_point frame_timestamp(
      std::chrono::nanoseconds(frame.monotonic_timestamp_ns()));

  std::chrono::duration<double> time(frame_timestamp -
                                     first_frame_monotonic_.value());

  // TODO: maybe this should not be hardcoded
  const int device_id = 1;

  // EFF/SFF is set in the MSB
  bool is_extended_frame_format = frame.can_id() & CAN_EFF_FLAG;

  uint32_t id_mask = is_extended_frame_format ? CAN_EFF_MASK : CAN_SFF_MASK;
  int id = frame.can_id() & id_mask;

  // data length code
  int dlc = can_fd_len2dlc(frame.data()->size());

  const uint8_t flags = frame.flags();

  uint32_t asc_flags = 0;

  // Mark it as a CAN FD Frame
  asc_flags = ASC_F_FDF;

  // Pass through the bit rate switch flag
  // indicates that it used a second bitrate for payload data
  if (flags & CANFD_BRS) {
    asc_flags |= ASC_F_BRS;
  }

  // ESI is the error state indicator of the transmitting node
  if (flags & CANFD_ESI) {
    asc_flags |= ASC_F_ESI;
  }

  file << std::fixed << time.count() << " ";

  file << "CANFD ";

  file << std::setfill(' ') << std::setw(3) << std::right << device_id << " ";

  file << "Rx ";

  std::stringstream formatted_id;
  formatted_id << std::hex << std::uppercase << std::setfill('0') << id
               << std::dec;
  if (is_extended_frame_format) {
    formatted_id << "x";
  }

  file << std::setfill(' ') << std::setw(11) << formatted_id.str();
  file << "                                  ";

  file << ((flags & CANFD_BRS) ? '1' : '0') << " ";
  file << ((flags & CANFD_ESI) ? '1' : '0') << " ";

  file << std::hex << std::nouppercase << dlc << std::dec << " ";

  // actual data length
  file << std::setfill(' ') << std::setw(2) << frame.data()->size();

  file << std::hex << std::uppercase;
  for (uint8_t byte : *frame.data()) {
    file << " " << std::setfill('0') << std::setw(2) << static_cast<int>(byte);
  }
  file << std::dec;

  // these are hardcoded in log2asc too, I don't know why
  file << "   130000  130 ";
  file << std::setfill(' ') << std::setw(8) << std::hex << asc_flags
       << std::dec;
  file << " 0 0 0 0 0";

  file << "\n";
}

}  // namespace can_logger
}  // namespace frc971
