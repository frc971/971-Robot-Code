#include "y2023/control_loops/drivetrain/target_selector.h"

#include "frc971/input/joystick_state_generated.h"
#include "gtest/gtest.h"
#include "y2023/constants/simulated_constants_sender.h"

using Side = frc971::control_loops::drivetrain::RobotSide;

namespace y2023::control_loops::drivetrain {
class TargetSelectorTest : public ::testing::Test {
 protected:
  TargetSelectorTest()
      : configuration_(aos::configuration::ReadConfig("y2023/aos_config.json")),
        event_loop_factory_(&configuration_.message()),
        roborio_node_([this]() {
          // Get the constants sent before anything else happens.
          // It has nothing to do with the roborio node.
          SendSimulationConstants(&event_loop_factory_, 7971,
                                  "y2023/constants/test_constants.json");
          return aos::configuration::GetNode(&configuration_.message(),
                                             "roborio");
        }()),
        selector_event_loop_(
            event_loop_factory_.MakeEventLoop("drivetrain", roborio_node_)),
        target_selector_(selector_event_loop_.get()),
        test_event_loop_(
            event_loop_factory_.MakeEventLoop("test", roborio_node_)),
        constants_fetcher_(test_event_loop_.get()),
        joystick_state_sender_(
            test_event_loop_->MakeSender<aos::JoystickState>("/aos")),
        hint_sender_(
            test_event_loop_->MakeSender<TargetSelectorHint>("/drivetrain")) {}

  void SendJoystickState() {
    auto builder = joystick_state_sender_.MakeBuilder();
    aos::JoystickState::Builder state_builder =
        builder.MakeBuilder<aos::JoystickState>();
    state_builder.add_alliance(aos::Alliance::kRed);
    builder.CheckOk(builder.Send(state_builder.Finish()));
  }

  void SendHint(GridSelectionHint grid, RowSelectionHint row,
                SpotSelectionHint spot, Side side) {
    auto builder = hint_sender_.MakeBuilder();
    builder.CheckOk(builder.Send(
        CreateTargetSelectorHint(*builder.fbb(), grid, row, spot, side)));
  }
  void SendHint(RowSelectionHint row, SpotSelectionHint spot, Side side) {
    auto builder = hint_sender_.MakeBuilder();
    TargetSelectorHint::Builder hint_builder =
        builder.MakeBuilder<TargetSelectorHint>();
    hint_builder.add_row(row);
    hint_builder.add_spot(spot);
    hint_builder.add_robot_side(side);
    builder.CheckOk(builder.Send(hint_builder.Finish()));
  }

  const localizer::HalfField *scoring_map() const {
    return constants_fetcher_.constants().scoring_map()->red();
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> configuration_;
  aos::SimulatedEventLoopFactory event_loop_factory_;
  const aos::Node *const roborio_node_;
  std::unique_ptr<aos::EventLoop> selector_event_loop_;
  TargetSelector target_selector_;
  std::unique_ptr<aos::EventLoop> test_event_loop_;
  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;
  aos::Sender<aos::JoystickState> joystick_state_sender_;
  aos::Sender<TargetSelectorHint> hint_sender_;
};

// Tests that no target is available if no input messages have been sent.
TEST_F(TargetSelectorTest, NoTargetWithoutInputs) {
  EXPECT_FALSE(target_selector_.UpdateSelection(
      Eigen::Matrix<double, 5, 1>::Zero(), 0.0));
  EXPECT_DEATH(target_selector_.TargetPose(), "Did you check the return value");
  EXPECT_EQ(0.0, target_selector_.TargetRadius());
}

// Tests that if we fully specify which target to go to that we always will do
// so.
TEST_F(TargetSelectorTest, FullySpecifiedTarget) {
  SendJoystickState();
  // Iterate over every available target.
  for (const auto &[grid, grid_hint] : std::vector<
           std::pair<const localizer::ScoringGrid *, GridSelectionHint>>{
           {scoring_map()->left_grid(), GridSelectionHint::LEFT},
           {scoring_map()->middle_grid(), GridSelectionHint::MIDDLE},
           {scoring_map()->right_grid(), GridSelectionHint::RIGHT}}) {
    for (const auto &[row, row_hint] : std::vector<
             std::pair<const localizer::ScoringRow *, RowSelectionHint>>{
             {grid->bottom(), RowSelectionHint::BOTTOM},
             {grid->middle(), RowSelectionHint::MIDDLE},
             {grid->top(), RowSelectionHint::TOP}}) {
      for (const auto &[spot, spot_hint] : std::vector<
               std::pair<const frc971::vision::Position *, SpotSelectionHint>>{
               {row->left_cone(), SpotSelectionHint::LEFT},
               {row->cube(), SpotSelectionHint::MIDDLE},
               {row->right_cone(), SpotSelectionHint::RIGHT}}) {
        SendHint(grid_hint, row_hint, spot_hint, Side::FRONT);
        EXPECT_TRUE(target_selector_.UpdateSelection(
            Eigen::Matrix<double, 5, 1>::Zero(), 0.0));
        EXPECT_EQ(0.0, target_selector_.TargetRadius());
        EXPECT_EQ(spot->x(), target_selector_.TargetPose().abs_pos().x());
        EXPECT_EQ(spot->y(), target_selector_.TargetPose().abs_pos().y());
        EXPECT_EQ(spot->z(), target_selector_.TargetPose().abs_pos().z());
        EXPECT_EQ(frc971::control_loops::drivetrain::TargetSelectorInterface::
                      Side::FRONT,
                  target_selector_.DriveDirection());
      }
    }
  }
}

// Tests that if we leave the grid setting ambiguous that we select the
// nearest possible target given the other settings.
TEST_F(TargetSelectorTest, NoGridSpecified) {
  SendJoystickState();
  // We will leave the robot at (0, 0). This means that if we are going for the
  // left cone we should go for the middle grid, and if we are going for the
  // cube (middle) or right cone positions we should prefer the left grid.
  // Note that the grids are not centered on the field (hence the middle isn't
  // always preferred when at (0, 0)).

  for (const auto &[spot, spot_hint] : std::vector<
           std::pair<const frc971::vision::Position *, SpotSelectionHint>>{
           {scoring_map()->middle_grid()->bottom()->left_cone(),
            SpotSelectionHint::LEFT},
           {scoring_map()->left_grid()->bottom()->cube(),
            SpotSelectionHint::MIDDLE},
           {scoring_map()->left_grid()->bottom()->right_cone(),
            SpotSelectionHint::RIGHT}}) {
    SendHint(RowSelectionHint::BOTTOM, spot_hint, Side::BACK);
    EXPECT_TRUE(target_selector_.UpdateSelection(
        Eigen::Matrix<double, 5, 1>::Zero(), 0.0));
    EXPECT_EQ(spot->x(), target_selector_.TargetPose().abs_pos().x());
    EXPECT_EQ(spot->y(), target_selector_.TargetPose().abs_pos().y());
    EXPECT_EQ(spot->z(), target_selector_.TargetPose().abs_pos().z());
    EXPECT_EQ(
        frc971::control_loops::drivetrain::TargetSelectorInterface::Side::BACK,
        target_selector_.DriveDirection());
  }
}

}  // namespace y2023::control_loops::drivetrain
