#include "y2019/jevois/serial.h"

#include "aos/logging/implementations.h"
#include "aos/logging/logging.h"
#include "y2019/jevois/cobs.h"
#include "y2019/jevois/serial.h"
#include "y2019/jevois/structures.h"
#include "y2019/jevois/uart.h"

#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

namespace y2019 {
namespace vision {

void main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  using namespace y2019::vision;
  using namespace frc971::jevois;
  // gflags::ParseCommandLineFlags(&argc, &argv, false);
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stderr));

  int flags = fcntl(0, F_GETFL, 0);
  fcntl(0, F_SETFL, flags | O_NONBLOCK);

  int itsDev = ::y2019::jevois::open_via_terminos("/dev/ttyUSB0");

  CobsPacketizer<uart_to_teensy_size()> cobs;
  while (true) {
    {
      constexpr size_t kBufferSize = uart_to_teensy_size();
      char data[kBufferSize];
      ssize_t n = read(itsDev, &data[0], kBufferSize);
      if (n >= 1) {
        cobs.ParseData(gsl::span<const char>(&data[0], n));
        auto packet = cobs.received_packet();
        if (!packet.empty()) {
          // One we read data from the serial, Teensy code will return an
          // optional with success if there is a new frame. unwrap that
          // and print out any frames from inside.
          auto frame_optional = UartUnpackToTeensy(packet);
          if (frame_optional) {
            printf("----------\n");
            const auto &frame = *frame_optional;
            for (const auto &target : frame.targets) {
              printf("z: %g y: %g, r1: %g, r2: %g\n", target.distance / 0.0254,
                     target.height / 0.0254, target.skew, target.heading);
            }
          } else {
            printf("bad frame\n");
          }
          cobs.clear_received_packet();
        }
      }
    }
    {
      constexpr size_t kBufferSize = 1024;
      char data[kBufferSize];
      // read command char from stdin. 'p' = passthrough else usb.
      ssize_t n = read(0, &data[0], kBufferSize);
      if (n >= 1) {
        CameraCalibration calibration{};
        if (data[0] == 'p') {
          calibration.camera_command = CameraCommand::kCameraPassthrough;
        } else {
          calibration.camera_command = CameraCommand::kUsb;
        }
        if (write(itsDev, "\0", 1) == 1) {
          const auto out_data = frc971::jevois::UartPackToCamera(calibration);
          // We don't really care if this succeeds or not. If it fails for some
          // reason, we'll just try again with the next frame, and the other end
          // will find the new packet just fine.
          if (static_cast<int>(
                  write(itsDev, out_data.data(), out_data.size())) !=
              static_cast<int>(out_data.size())) {
            printf("ERROR!!! START OVER FROM FIRST PRINCIPLES!!\n");
          }
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

}  // namespace y2019
}  // namespace vision

int main(int argc, char **argv) { y2019::vision::main(argc, argv); }
