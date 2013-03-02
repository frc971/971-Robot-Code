#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/index_motor.q.h"
#include "frc971/control_loops/index.h"
#include "frc971/control_loops/index_motor_plant.h"
#include "frc971/control_loops/transfer_motor_plant.h"
#include "frc971/constants.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {

// TODO(aschuh): Figure out these constants.
const double kTransferStartPosition = 0.0;
const double kIndexStartPosition = 0.5;
const double kIndexStopPosition = 2.5;
const double kGrabberStopPosition = 2.625;
const double kGrabberMovementVelocity = 0.4;

// Start and stop position of the bottom disc detect sensor in meters.
const double kBottomDiscDetectStart = -0.08;
const double kBottomDiscDetectStop = 0.200025;

const double kTopDiscDetectStart = 18.0;
const double kTopDiscDetectStop = 19.0;

// Disc radius in meters.
const double kDiscRadius = 11.875 * 0.0254 / 2;
// Roller radius in meters.
const double kRollerRadius = 2.0 * 0.0254 / 2;

class Frisbee {
 public:
  // Creates a frisbee starting at the specified position in the frisbee path,
  // and with the transfer and index rollers at the specified positions.
  Frisbee(double transfer_roller_position,
          double index_roller_position,
          double position = 0.0)
      : transfer_roller_position_(transfer_roller_position),
        index_roller_position_(index_roller_position),
        clamped_(false),
        position_(position) {
  }

  // Returns true if the frisbee is controlled by the transfer roller.
  bool IsTouchingTransfer() const {
    return (position_ >= kTransferStartPosition &&
            position_ <= kIndexStartPosition);
  }

  // Returns true if the frisbee is controlled by the indexing roller.
  bool IsTouchingIndex() const {
    return (position_ >= kIndexStartPosition &&
            position_ <= kIndexStopPosition);
  }

  // Returns true if the frisbee is in a position such that the grabber will
  // pull it into the loader.
  bool IsTouchingGrabber() const {
    return (position_ >= kIndexStopPosition &&
            position_ <= kGrabberStopPosition);
  }

  // Returns true if the disc is triggering the bottom disc detect sensor.
  bool bottom_disc_detect() const {
    return (position_ >= kBottomDiscDetectStart &&
            position_ <= kBottomDiscDetectStop);
  }

  // Returns true if the disc is triggering the top disc detect sensor.
  bool top_disc_detect() const {
    return (position_ >= kTopDiscDetectStart &&
            position_ <= kTopDiscDetectStop);
  }

  // Converts the angle of the indexer to the distance traveled by the center of
  // the disc.
  double ConvertIndexToDiscPosition(const double angle) const {
    return (angle * (kDiscRadius + kRollerRadius) /
            (1 + (kDiscRadius * 2 + kRollerRadius) / kRollerRadius));
  }

  // Converts the angle of the transfer to the distance traveled by the center
  // of the disc.
  double ConvertTransferToDiscPosition(const double angle) const {
    return ConvertIndexToDiscPosition(angle);
  }

  // Updates the position of the frisbee in the frisbee path.
  void UpdatePosition(double transfer_roller_position,
                      double index_roller_position,
                      bool clamped) {
    // TODO(aschuh): Assert that you can't slide the frisbee through the
    // clamp.
    if (IsTouchingTransfer()) {
      position_ += ConvertTransferToDiscPosition(transfer_roller_position -
                                                 transfer_roller_position_);
    } else if (IsTouchingIndex()) {
      position_ += ConvertIndexToDiscPosition(index_roller_position -
                                              index_roller_position_);
    } else if (IsTouchingGrabber()) {
      if (clamped) {
        position_ = ::std::min(position_ + kGrabberMovementVelocity / 100.0,
                               kGrabberStopPosition);
      }
    } else {
      // TODO(aschuh): Deal with lifting.
      // TODO(aschuh): Deal with shooting.
      // We must wait long enough for the disc to leave the loader before
      // lowering.
    }
    transfer_roller_position_ = transfer_roller_position;
    index_roller_position_ = index_roller_position;
    clamped_ = clamped;
    printf("Disc is at %f\n", position_);
  }

  double position() const {
    return position_;
  }

 private:
  double transfer_roller_position_;
  double index_roller_position_;
  bool clamped_;
  double position_;
};


// Class which simulates the index and sends out queue messages containing the
// position.
class IndexMotorSimulation {
 public:
  // Constructs a motor simulation.  initial_position is the inital angle of the
  // index, which will be treated as 0 by the encoder.
  IndexMotorSimulation()
      : index_plant_(new StateFeedbackPlant<2, 1, 1>(MakeIndexPlant())),
        transfer_plant_(new StateFeedbackPlant<2, 1, 1>(MakeTransferPlant())),
        my_index_loop_(".frc971.control_loops.index",
                       0x1a7b7094, ".frc971.control_loops.index.goal",
                       ".frc971.control_loops.index.position",
                       ".frc971.control_loops.index.output",
                       ".frc971.control_loops.index.status") {
  }

  // Starts a disc at the start of the index.
  void InsertDisc() {
    frisbees.push_back(Frisbee(transfer_roller_position(),
                               index_roller_position()));
  }

  // Returns true if the bottom disc sensor is triggered.
  bool BottomDiscDetect() const {
    bool bottom_disc_detect = false;
    for (const Frisbee &frisbee : frisbees) {
      bottom_disc_detect |= frisbee.bottom_disc_detect();
    }
    return bottom_disc_detect;
  }

  // Returns true if the top disc sensor is triggered.
  bool TopDiscDetect() const {
    bool top_disc_detect = false;
    for (const Frisbee &frisbee : frisbees) {
      top_disc_detect |= frisbee.top_disc_detect();
    }
    return top_disc_detect;
  }

  void UpdateDiscs(bool clamped) {
    for (Frisbee &frisbee : frisbees) {
      // TODO(aschuh): Simulate clamping
      frisbee.UpdatePosition(transfer_roller_position(),
                             index_roller_position(),
                             clamped);
    }
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::IndexLoop::Position> position =
        my_index_loop_.position.MakeMessage();
    position->index_position = index_roller_position();
    position->bottom_disc_detect = BottomDiscDetect();
    position->top_disc_detect = TopDiscDetect();
    printf("bdd: %x tdd: %x\n", position->bottom_disc_detect,
           position->top_disc_detect);
    position.Send();
  }

  // Simulates the index moving for one timestep.
  void Simulate() {
    EXPECT_TRUE(my_index_loop_.output.FetchLatest());

    index_plant_->U << my_index_loop_.output->index_voltage;
    index_plant_->Update();

    transfer_plant_->U << my_index_loop_.output->transfer_voltage;
    transfer_plant_->Update();
    printf("tv: %f iv: %f tp : %f ip: %f\n",
           my_index_loop_.output->transfer_voltage,
           my_index_loop_.output->index_voltage,
           transfer_roller_position(), index_roller_position());

    UpdateDiscs(my_index_loop_.output->disc_clamped);
  }

  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> index_plant_;
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> transfer_plant_;

  // Returns the absolute angle of the index.
  double index_roller_position() const {
    return index_plant_->Y(0, 0);
  }

  // Returns the absolute angle of the index.
  double transfer_roller_position() const {
    return transfer_plant_->Y(0, 0);
  }

  ::std::vector<Frisbee> frisbees;

 private:
  IndexLoop my_index_loop_;
};


class IndexTest : public ::testing::Test {
 protected:
  IndexTest() : my_index_loop_(".frc971.control_loops.index",
                               0x1a7b7094, ".frc971.control_loops.index.goal",
                               ".frc971.control_loops.index.position",
                               ".frc971.control_loops.index.output",
                               ".frc971.control_loops.index.status"),
                index_motor_(&my_index_loop_),
                index_motor_plant_(),
                loop_count_(0) {
    // Flush the robot state queue so we can use clean shared memory for this
    // test.
    ::aos::robot_state.Clear();
    SendDSPacket(true);
    Time::EnableMockTime(Time(0, 0));
  }

  virtual ~IndexTest() {
    ::aos::robot_state.Clear();
    Time::DisableMockTime();
  }

  // Sends a DS packet with the enable bit set to enabled.
  void SendDSPacket(bool enabled) {
    ::aos::robot_state.MakeWithBuilder().enabled(enabled)
                                        .autonomous(false)
                                        .team_id(971).Send();
    ::aos::robot_state.FetchLatest();
  }

  // Updates the current mock time.
  void UpdateTime() {
    loop_count_ += 1;
    Time::SetMockTime(Time::InMS(10 * loop_count_));
  }

  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  IndexLoop my_index_loop_;

  // Create a loop and simulation plant.
  IndexMotor index_motor_;
  IndexMotorSimulation index_motor_plant_;

  int loop_count_;
};

// Tests that the index grabs 1 disc and places it at the correct position.
TEST_F(IndexTest, GrabSingleDisc) {
  my_index_loop_.goal.MakeWithBuilder().goal_state(2).Send();
  for (int i = 0; i < 250; ++i) {
    index_motor_plant_.SendPositionMessage();
    index_motor_.Iterate();
    if (i == 100) {
      EXPECT_EQ(0, index_motor_plant_.index_roller_position());
      index_motor_plant_.InsertDisc();
    }
    index_motor_plant_.Simulate();
    SendDSPacket(true);
    UpdateTime();
  }

  EXPECT_TRUE(my_index_loop_.status.FetchLatest());
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 1);
  EXPECT_EQ(static_cast<size_t>(1), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(
      kIndexStartPosition + IndexMotor::ConvertDiscAngleToDiscPosition(M_PI),
      index_motor_plant_.frisbees[0].position(), 0.01);
}

// Test that pulling in a second disc works correctly.
// Test that HOLD still finishes the disc correctly.
// Test that pulling a disc down works correctly and ready_to_intake waits.

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
