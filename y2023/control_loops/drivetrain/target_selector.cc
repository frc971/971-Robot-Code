#include "y2023/control_loops/drivetrain/target_selector.h"

#include "aos/containers/sized_array.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "y2023/control_loops/superstructure/superstructure_position_generated.h"
#include "y2023/vision/game_pieces_generated.h"

namespace y2023::control_loops::drivetrain {
namespace {
// If we already have a target selected, require the robot to be closer than
// this distance (in meters) to one target than another before swapping.
constexpr double kGridHysteresisDistance = 0.1;
}  // namespace

TargetSelector::TargetSelector(aos::EventLoop *event_loop)
    : joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      hint_fetcher_(event_loop->MakeFetcher<TargetSelectorHint>("/drivetrain")),
      superstructure_status_fetcher_(
          event_loop->MakeFetcher<superstructure::Status>("/superstructure")),
      status_sender_(
          event_loop->MakeSender<TargetSelectorStatus>("/drivetrain")),
      constants_fetcher_(event_loop) {
  CHECK(constants_fetcher_.constants().has_scoring_map());
  CHECK(constants_fetcher_.constants().scoring_map()->has_red());
  CHECK(constants_fetcher_.constants().scoring_map()->has_blue());

  event_loop->AddPhasedLoop(
      [this](int) {
        auto builder = status_sender_.MakeBuilder();
        auto status_builder = builder.MakeBuilder<TargetSelectorStatus>();
        status_builder.add_game_piece_position(game_piece_position_);
        builder.CheckOk(builder.Send(status_builder.Finish()));
      },
      std::chrono::milliseconds(100));
}

void TargetSelector::UpdateAlliance() {
  joystick_state_fetcher_.Fetch();
  if (joystick_state_fetcher_.get() != nullptr &&
      joystick_state_fetcher_->has_alliance()) {
    switch (joystick_state_fetcher_->alliance()) {
      case aos::Alliance::kRed:
        scoring_map_ = constants_fetcher_.constants().scoring_map()->red();
        break;
      case aos::Alliance::kBlue:
        scoring_map_ = constants_fetcher_.constants().scoring_map()->blue();
        break;
      case aos::Alliance::kInvalid:
        // Do nothing.
        break;
    }
  }
}

bool TargetSelector::UpdateSelection(const ::Eigen::Matrix<double, 5, 1> &state,
                                     double /*command_speed*/) {
  UpdateAlliance();
  if (scoring_map_ == nullptr) {
    // We don't know which alliance we are on yet; wait on a JoystickState
    // message.
    return false;
  }
  hint_fetcher_.Fetch();
  if (hint_fetcher_.get() == nullptr) {
    // We don't know where to go, wait on a hint.
    return false;
  }
  // Keep track of when the hint changes (note that this will not detect default
  // vs. not populated default values); when it changes, force us to reselect
  // the target.
  {
    TargetSelectorHintT hint_object;
    hint_fetcher_.get()->UnPackTo(&hint_object);
    if (!last_hint_.has_value() || hint_object != last_hint_) {
      target_pose_.reset();
    }
    last_hint_ = hint_object;
  }
  aos::SizedArray<const localizer::ScoringGrid *, 3> possible_grids;
  if (hint_fetcher_->has_grid()) {
    possible_grids = {[this]() -> const localizer::ScoringGrid * {
      switch (hint_fetcher_->grid()) {
        case GridSelectionHint::LEFT:
          return scoring_map_->left_grid();
        case GridSelectionHint::MIDDLE:
          return scoring_map_->middle_grid();
        case GridSelectionHint::RIGHT:
          return scoring_map_->right_grid();
      }
      // Make roborio compiler happy...
      return nullptr;
    }()};
  } else {
    possible_grids = {scoring_map_->left_grid(), scoring_map_->middle_grid(),
                      scoring_map_->right_grid()};
  }

  aos::SizedArray<const localizer::ScoringRow *, 3> possible_rows =
      [this, possible_grids]() {
        aos::SizedArray<const localizer::ScoringRow *, 3> rows;
        for (const localizer::ScoringGrid *grid : possible_grids) {
          CHECK_NOTNULL(grid);
          switch (hint_fetcher_->row()) {
            case RowSelectionHint::BOTTOM:
              rows.push_back(grid->bottom());
              break;
            case RowSelectionHint::MIDDLE:
              rows.push_back(grid->middle());
              break;
            case RowSelectionHint::TOP:
              rows.push_back(grid->top());
              break;
          }
        }
        return rows;
      }();
  aos::SizedArray<const frc971::vision::Position *, 3> possible_positions =
      [this, possible_rows]() {
        aos::SizedArray<const frc971::vision::Position *, 3> positions;
        for (const localizer::ScoringRow *row : possible_rows) {
          CHECK_NOTNULL(row);
          switch (hint_fetcher_->spot()) {
            case SpotSelectionHint::LEFT:
              positions.push_back(row->left_cone());
              break;
            case SpotSelectionHint::MIDDLE:
              positions.push_back(row->cube());
              break;
            case SpotSelectionHint::RIGHT:
              positions.push_back(row->right_cone());
              break;
          }
        }
        return positions;
      }();
  CHECK_LT(0u, possible_positions.size());
  aos::SizedArray<double, 3> distances;
  std::optional<double> closest_distance;
  std::optional<Eigen::Vector3d> closest_position;
  const Eigen::Vector3d robot_position(state.x(), state.y(), 0.0);
  for (const frc971::vision::Position *position : possible_positions) {
    const Eigen::Vector3d target(position->x(), position->y(), position->z());
    double distance = (target - robot_position).norm();
    distances.push_back(distance);
    if (!closest_distance.has_value() || distance < closest_distance.value()) {
      closest_distance = distance;
      closest_position = target;
    }
  }
  std::sort(distances.begin(), distances.end());
  CHECK_EQ(distances.at(0), closest_distance.value());
  // Only change the target pose if one grid is clearly better than the other.
  // This prevents us from dithering between two grids if we happen to be on the
  // boundary.
  if (!target_pose_.has_value() ||
      distances.at(1) - distances.at(0) > kGridHysteresisDistance) {
    CHECK(closest_position.has_value());
    target_pose_ = Pose(closest_position.value(), /*theta=*/0.0);
    if (hint_fetcher_->has_robot_side()) {
      drive_direction_ = hint_fetcher_->robot_side();
    } else {
      drive_direction_ = Side::DONT_CARE;
    }
    // Only update the game piece position when we reassign the target.
    superstructure_status_fetcher_.Fetch();
    if (superstructure_status_fetcher_.get() != nullptr) {
      game_piece_position_ =
          superstructure_status_fetcher_->game_piece_position();
    }
  }
  CHECK(target_pose_.has_value());
  return true;
}

}  // namespace y2023::control_loops::drivetrain
