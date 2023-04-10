#include "y2023/vision/game_pieces.h"

#include <ctime>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "frc971/vision/vision_generated.h"
#include "y2023/vision/yolov5.h"

// The best_x and best_y are pixel (x, y) cordinates. The 'best'
// game piece is picked on proximity to the specified cordinates.
// The cordinate should represent where we want to intake a game piece.
// (0, 360) was chosen without any testing, just a cordinate that
// seemed reasonable.

DEFINE_uint32(
    best_x, 0,
    "The 'best' game piece is picked based on how close it is to this x value");

DEFINE_uint32(
    best_y, 360,
    "The 'best' game piece is picked based on how close it is to this y value");

namespace y2023 {
namespace vision {
using aos::monotonic_clock;
GamePiecesDetector::GamePiecesDetector(aos::EventLoop *event_loop)
    : game_pieces_sender_(event_loop->MakeSender<GamePieces>("/camera")) {
  model = MakeYOLOV5();
  model->LoadModel("edgetpu_model.tflite");

  event_loop->MakeWatcher(
      "/camera", [this, event_loop](const CameraImage &camera_image) {
        const monotonic_clock::time_point eof = monotonic_clock::time_point(
            std::chrono::nanoseconds(camera_image.monotonic_timestamp_ns()));
        const monotonic_clock::duration age = event_loop->monotonic_now() - eof;
        if (age > std::chrono::milliseconds(100)) {
          VLOG(1) << "Behind, skipping";
          return;
        }

        this->ProcessImage(camera_image);
      });
}

void GamePiecesDetector::ProcessImage(const CameraImage &image) {
  cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                          (void *)image.data()->data());
  std::vector<Detection> detections;
  cv::Mat image_mat(cv::Size(image.cols(), image.rows()), CV_8UC3);
  cv::cvtColor(image_color_mat, image_mat, cv::COLOR_YUV2BGR_YUYV);

  detections = model->ProcessImage(image_mat);

  auto builder = game_pieces_sender_.MakeBuilder();

  std::vector<flatbuffers::Offset<GamePiece>> game_pieces_offsets;

  float lowest_distance = std::numeric_limits<float>::max();
  int best_distance_index = 0;

  for (size_t i = 0; i < detections.size(); i++) {
    auto box_builder = builder.MakeBuilder<Box>();
    box_builder.add_h(detections[i].box.height);
    box_builder.add_w(detections[i].box.width);
    box_builder.add_x(detections[i].box.x);
    box_builder.add_y(detections[i].box.y);
    auto box_offset = box_builder.Finish();

    auto game_piece_builder = builder.MakeBuilder<GamePiece>();
    switch (detections[i].class_id) {
      case 0:
        game_piece_builder.add_piece_class(Class::CONE_DOWN);
        break;
      case 1:
        game_piece_builder.add_piece_class(Class::CONE_UP);
        break;
      case 2:
        game_piece_builder.add_piece_class(Class::CUBE);
        break;
      default:
        game_piece_builder.add_piece_class(Class::CONE_DOWN);
    }
    game_piece_builder.add_box(box_offset);
    game_piece_builder.add_confidence(detections[i].confidence);
    auto game_piece = game_piece_builder.Finish();
    game_pieces_offsets.push_back(game_piece);

    // Center x and y values.
    // Inference returns the top left corner of the bounding box
    // but we want the center of the box for this.

    const int center_x = detections[i].box.x + detections[i].box.width / 2;
    const int center_y = detections[i].box.y + detections[i].box.height / 2;

    // Find difference between target x, y and the x, y
    // of the bounding box using Euclidean distance.

    const int dx = FLAGS_best_x - center_x;
    const int dy = FLAGS_best_y - center_y;
    const float distance = std::sqrt(dx * dx + dy * dy);

    if (distance < lowest_distance) {
      lowest_distance = distance;
      best_distance_index = i;
    }
  };

  auto game_pieces_vector = builder.fbb()->CreateVector(game_pieces_offsets);

  auto game_pieces_builder = builder.MakeBuilder<GamePieces>();
  game_pieces_builder.add_game_pieces(game_pieces_vector);
  game_pieces_builder.add_best_piece(best_distance_index);

  builder.CheckOk(builder.Send(game_pieces_builder.Finish()));
}

}  // namespace vision
}  // namespace y2023
