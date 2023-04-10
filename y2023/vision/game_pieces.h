#ifndef Y2023_VISION_GAME_PIECES_H_
#define Y2023_VISION_GAME_PIECES_H_

#include "aos/events/shm_event_loop.h"
#include "frc971/vision/vision_generated.h"
#include "y2023/vision/game_pieces_generated.h"

#include "y2023/vision/yolov5.h"

namespace y2023 {
namespace vision {

using namespace frc971::vision;

// Takes in camera images and detects game pieces in the image.
// Note: Actual detection has not been implemented yet.
class GamePiecesDetector {
 public:
  GamePiecesDetector(aos::EventLoop *event_loop);

  void ProcessImage(const CameraImage &camera_image);

 private:
  aos::Sender<GamePieces> game_pieces_sender_;
  std::unique_ptr<YOLOV5> model;
};
}  // namespace vision
}  // namespace y2023
#endif
