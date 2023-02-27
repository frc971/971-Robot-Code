#include "y2023/vision/game_pieces.h"

#include <ctime>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "frc971/vision/vision_generated.h"

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
GamePiecesDetector::GamePiecesDetector(aos::EventLoop *event_loop)
    : game_pieces_sender_(event_loop->MakeSender<GamePieces>("/camera")) {
  event_loop->MakeWatcher("/camera", [this](const CameraImage &camera_image) {
    this->ProcessImage(camera_image);
  });
}

// TODO(FILIP): Actually do inference.

void GamePiecesDetector::ProcessImage(const CameraImage &image) {
  // Param is not used for now.
  (void)image;

  const int detection_count = 5;

  auto builder = game_pieces_sender_.MakeBuilder();

  std::vector<flatbuffers::Offset<GamePiece>> game_pieces_offsets;

  float lowest_distance = std::numeric_limits<float>::max();
  int best_distance_index = 0;
  srand(time(0));

  for (int i = 0; i < detection_count; i++) {
    int h = rand() % 1000;
    int w = rand() % 1000;
    int x = rand() % 250;
    int y = rand() % 250;

    auto box_builder = builder.MakeBuilder<Box>();
    box_builder.add_h(h);
    box_builder.add_w(w);
    box_builder.add_x(x);
    box_builder.add_y(y);
    auto box_offset = box_builder.Finish();

    auto game_piece_builder = builder.MakeBuilder<GamePiece>();
    game_piece_builder.add_piece_class(y2023::vision::Class::CONE_DOWN);
    game_piece_builder.add_box(box_offset);
    game_piece_builder.add_confidence(0.9);
    auto game_piece = game_piece_builder.Finish();
    game_pieces_offsets.push_back(game_piece);

    // Center x and y values.
    // Inference returns the top left corner of the bounding box
    // but we want the center of the box for this.

    const int center_x = x + w / 2;
    const int center_y = y + h / 2;

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

  flatbuffers::FlatBufferBuilder fbb;
  auto game_pieces_vector = fbb.CreateVector(game_pieces_offsets);

  auto game_pieces_builder = builder.MakeBuilder<GamePieces>();
  game_pieces_builder.add_game_pieces(game_pieces_vector);
  game_pieces_builder.add_best_piece(best_distance_index);

  builder.CheckOk(builder.Send(game_pieces_builder.Finish()));
}

}  // namespace vision
}  // namespace y2023