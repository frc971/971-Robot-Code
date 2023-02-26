#include "y2023/vision/game_pieces.h"

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "frc971/vision/vision_generated.h"

namespace y2023 {
namespace vision {
GamePiecesDetector::GamePiecesDetector(aos::EventLoop *event_loop)
    : game_pieces_sender_(
          event_loop->MakeSender<GamePieces>("/camera")) {
  event_loop->MakeWatcher("/camera", [this](const CameraImage &camera_image) {
    this->ProcessImage(camera_image);
  });
}

void GamePiecesDetector::ProcessImage(const CameraImage &image) {
  // Param is not used for now.
  (void)image;

  auto builder = game_pieces_sender_.MakeBuilder();

  auto box_builder = builder.MakeBuilder<Box>();
  box_builder.add_h(10);
  box_builder.add_w(20);
  box_builder.add_x(30);
  box_builder.add_y(40);
  auto box_offset = box_builder.Finish();

  auto game_piece_builder = builder.MakeBuilder<GamePiece>();
  game_piece_builder.add_piece_class(y2023::vision::Class::CONE_DOWN);
  game_piece_builder.add_box(box_offset);
  game_piece_builder.add_confidence(0.9);
  auto game_piece = game_piece_builder.Finish();

  flatbuffers::FlatBufferBuilder fbb;
  auto game_pieces_vector =
      fbb.CreateVector(std::vector<flatbuffers::Offset<GamePiece>>{game_piece});

  auto game_pieces_builder = builder.MakeBuilder<GamePieces>();
  game_pieces_builder.add_game_pieces(game_pieces_vector);

  builder.CheckOk(builder.Send(game_pieces_builder.Finish()));
}

}  // namespace vision
}  // namespace y2023