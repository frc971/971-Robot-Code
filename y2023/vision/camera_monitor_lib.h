#ifndef Y2023_VISION_CAMERA_MONITOR_LIB_H_
#define Y2023_VISION_CAMERA_MONITOR_LIB_H_
#include "aos/events/event_loop.h"
#include "aos/starter/starter_rpc_lib.h"
#include "frc971/vision/vision_generated.h"
namespace y2023::vision {
// This class provides an application that will restart the camera_reader
// process whenever images stop flowing for too long. This is to mitigate an
// issue where sometimes we stop getting camera images.
class CameraMonitor {
 public:
  CameraMonitor(aos::EventLoop *event_loop);

 private:
  void SetImageTimeout();
  aos::EventLoop *event_loop_;
  aos::starter::StarterClient starter_;
  aos::TimerHandler *image_timeout_;
};
}  // namespace y2023::vision
#endif  // Y2023_VISION_CAMERA_MONITOR_LIB_H_
