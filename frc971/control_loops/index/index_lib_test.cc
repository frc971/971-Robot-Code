#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/index/index.h"
#include "frc971/control_loops/index/index_motor_plant.h"
#include "frc971/control_loops/index/transfer_motor_plant.h"
#include "frc971/constants.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {

class Frisbee {
 public:
  // Creates a frisbee starting at the specified position in the frisbee path,
  // and with the transfer and index rollers at the specified positions.
  Frisbee(double transfer_roller_position,
          double index_roller_position,
          double position = IndexMotor::kBottomDiscDetectStart - 0.001)
      : transfer_roller_position_(transfer_roller_position),
        index_roller_position_(index_roller_position),
        position_(position),
        has_been_shot_(false),
        has_bottom_disc_negedge_wait_position_(false),
        bottom_disc_negedge_wait_position_(0.0),
        after_negedge_time_left_(IndexMotor::kBottomDiscIndexDelay),
        counted_negedge_wait_(false),
        has_top_disc_posedge_position_(false),
        top_disc_posedge_position_(0.0) {
  }

  // Returns true if the frisbee is controlled by the transfer roller.
  bool IsTouchingTransfer(double position) const {
    return (position >= IndexMotor::kBottomDiscDetectStart &&
            position <= IndexMotor::kIndexStartPosition);
  }
  bool IsTouchingTransfer() const { return IsTouchingTransfer(position_); }

  // Returns true if the frisbee is in a place where it is unsafe to grab.
  bool IsUnsafeToGrab() const {
    return (position_ > (IndexMotor::kLoaderFreeStopPosition) &&
            position_ < IndexMotor::kGrabberStartPosition);
  }

  // Returns true if the frisbee is controlled by the indexing roller.
  bool IsTouchingIndex(double position) const {
    return (position >= IndexMotor::kIndexStartPosition &&
            position < IndexMotor::kGrabberStartPosition);
  }
  bool IsTouchingIndex() const { return IsTouchingIndex(position_); }

  // Returns true if the frisbee is in a position such that the disc can be
  // lifted.
  bool IsUnsafeToLift() const {
    return (position_ >= IndexMotor::kLoaderFreeStopPosition &&
            position_ <= IndexMotor::kReadyToLiftPosition);
  }

  // Returns true if the frisbee is in a position such that the grabber will
  // pull it into the loader.
  bool IsTouchingGrabber() const {
    return (position_ >= IndexMotor::kGrabberStartPosition &&
            position_ < IndexMotor::kReadyToLiftPosition);
  }

  // Returns true if the frisbee is in a position such that the disc can be
  // lifted.
  bool IsTouchingLoader() const {
    return (position_ >= IndexMotor::kReadyToLiftPosition &&
            position_ < IndexMotor::kLifterStopPosition);
  }

  // Returns true if the frisbee is touching the ejector.
  bool IsTouchingEjector() const {
    return (position_ >= IndexMotor::kLifterStopPosition &&
            position_ < IndexMotor::kEjectorStopPosition);
  }

  // Returns true if the disc is triggering the bottom disc detect sensor.
  bool bottom_disc_detect(double position) const {
    return (position >= IndexMotor::kBottomDiscDetectStart &&
            position <= IndexMotor::kBottomDiscDetectStop);
  }
  bool bottom_disc_detect() const { return bottom_disc_detect(position_); }

  // Returns true if the disc is triggering the top disc detect sensor.
  bool top_disc_detect(double position) const {
    return (position >= IndexMotor::kTopDiscDetectStart &&
            position <= IndexMotor::kTopDiscDetectStop);
  }
  bool top_disc_detect() const { return top_disc_detect(position_); }

  // Returns true if the bottom disc sensor will negedge after the disc moves
  // by dx.
  bool will_negedge_bottom_disc_detect(double disc_dx) {
    if (bottom_disc_detect()) {
      return !bottom_disc_detect(position_ + disc_dx);
    }
    return false;
  }

  // Returns true if the bottom disc sensor will negedge after the disc moves
  // by dx.
  bool will_posedge_top_disc_detect(double disc_dx) {
    if (!top_disc_detect()) {
      return top_disc_detect(position_ + disc_dx);
    }
    return false;
  }

  // Handles potentially dealing with the delayed negedge.
  // Computes the index position when time expires using the cached old indexer
  // position, the elapsed time, and the average velocity.
  void HandleAfterNegedge(
      double index_velocity, double elapsed_time, double time_left) {
    if (!has_bottom_disc_negedge_wait_position_) {
      if (time_left < after_negedge_time_left_) {
        after_negedge_time_left_ = 0.0;
        // Assume constant velocity and compute the position.
        bottom_disc_negedge_wait_position_ =
            index_roller_position_ +
            index_velocity * (elapsed_time + after_negedge_time_left_);
        has_bottom_disc_negedge_wait_position_ = true;
      } else {
        after_negedge_time_left_ -= elapsed_time;
      }
    }
  }

  // Updates the position of the disc assuming that it has started on the
  // transfer.  The elapsed time is the simulated amount of time that has
  // elapsed since the simulation timestep started and this method was called.
  // time_left is the amount of time left to spend during this timestep.
  double UpdateTransferPositionForTime(double transfer_roller_velocity,
                                       double index_roller_velocity,
                                       double elapsed_time,
                                       double time_left) {
    double disc_dx = IndexMotor::ConvertTransferToDiscPosition(
        transfer_roller_velocity * time_left);
    bool shrunk_time = false;
    if (!IsTouchingTransfer(position_ + disc_dx)) {
      shrunk_time = true;
      time_left = (IndexMotor::kIndexStartPosition - position_) /
          transfer_roller_velocity;
      disc_dx = IndexMotor::ConvertTransferToDiscPosition(
          transfer_roller_velocity * time_left);
    }

    if (will_negedge_bottom_disc_detect(disc_dx)) {
      // Compute the time from the negedge to the end of the cycle assuming
      // constant velocity.
      const double elapsed_time =
          (position_ + disc_dx - IndexMotor::kBottomDiscDetectStop) /
          disc_dx * time_left;

      // I am not implementing very short delays until this fails.
      assert(elapsed_time <= after_negedge_time_left_);
      after_negedge_time_left_ -= elapsed_time;
    } else if (position_ >= IndexMotor::kBottomDiscDetectStop) {
      HandleAfterNegedge(index_roller_velocity, elapsed_time, time_left);
    }

    if (shrunk_time) {
      position_ = IndexMotor::kIndexStartPosition;
    } else {
      position_ += disc_dx;
    }
    printf("Transfer Roller: Disc is at %f\n", position_);
    return time_left;
  }

  // Updates the position of the disc assuming that it has started on the
  // indexer.  The elapsed time is the simulated amount of time that has
  // elapsed since the simulation timestep started and this method was called.
  // time_left is the amount of time left to spend during this timestep.
  double UpdateIndexPositionForTime(double index_roller_velocity,
                                    double elapsed_time,
                                    double time_left) {
    double index_dx = IndexMotor::ConvertIndexToDiscPosition(
        index_roller_velocity * time_left);
    bool shrunk_time = false;
    if (!IsTouchingIndex(position_ + index_dx)) {
      shrunk_time = true;
      time_left = (IndexMotor::kGrabberStartPosition - position_) /
          index_roller_velocity;
      index_dx = IndexMotor::ConvertTransferToDiscPosition(
          index_roller_velocity * time_left);
    }

    if (position_ >= IndexMotor::kBottomDiscDetectStop) {
      HandleAfterNegedge(index_roller_velocity, elapsed_time, time_left);
    }

    if (will_posedge_top_disc_detect(index_dx)) {
      // Wohoo!  Find the edge.
      // Assume constant velocity and compute the position.
      double edge_position = index_roller_velocity > 0 ?
          IndexMotor::kTopDiscDetectStart : IndexMotor::kTopDiscDetectStop;
      const double disc_time =
          (edge_position - position_) / index_roller_velocity;
      top_disc_posedge_position_ = index_roller_position_ +
          IndexMotor::ConvertDiscPositionToIndex(
          index_roller_velocity * (elapsed_time + disc_time));
      has_top_disc_posedge_position_ = true;
      printf("Posedge on top sensor at %f\n", top_disc_posedge_position_);
    }

    if (shrunk_time) {
      position_ = IndexMotor::kGrabberStartPosition;
    } else {
      position_ += index_dx;
    }
    printf("Index: Disc is at %f\n", position_);
    return time_left;
  }

  // Updates the position given velocities, piston comands, and the time left in
  // the simulation cycle.
  void UpdatePositionForTime(double transfer_roller_velocity,
                             double index_roller_velocity,
                             bool clamped,
                             bool lifted,
                             bool ejected,
                             double time_left) {
    double elapsed_time = 0.0;
    // We are making this assumption below
    ASSERT_LE(IndexMotor::kBottomDiscDetectStop,
              IndexMotor::kIndexStartPosition);
    if (IsTouchingTransfer() || position() < 0.0) {
      double deltat = UpdateTransferPositionForTime(
          transfer_roller_velocity, index_roller_velocity,
          elapsed_time, time_left);
      time_left -= deltat;
      elapsed_time += deltat;
    }

    if (IsTouchingIndex() && time_left >= 0) {
      // Verify that we aren't trying to grab or lift when it isn't safe.
      EXPECT_FALSE(clamped && IsUnsafeToGrab());
      EXPECT_FALSE(lifted && IsUnsafeToLift());

      double deltat = UpdateIndexPositionForTime(
          index_roller_velocity, elapsed_time, time_left);
      time_left -= deltat;
      elapsed_time += deltat;
    }
    if (IsTouchingGrabber()) {
      if (clamped) {
        const double grabber_dx =
            IndexMotor::kGrabberMovementVelocity * time_left;
        position_ = ::std::min(position_ + grabber_dx,
                               IndexMotor::kReadyToLiftPosition);
      }
      EXPECT_FALSE(lifted) << "Can't lift while in grabber";
      EXPECT_FALSE(ejected) << "Can't eject while in grabber";
      printf("Grabber: Disc is at %f\n", position_);
    } else if (IsTouchingLoader()) {
      if (lifted) {
        const double lifter_dx =
            IndexMotor::kLifterMovementVelocity * time_left;
        position_ = ::std::min(position_ + lifter_dx,
                               IndexMotor::kLifterStopPosition);
      }
      EXPECT_TRUE(clamped);
      EXPECT_FALSE(ejected);
      printf("Loader: Disc is at %f\n", position_);
    } else if (IsTouchingEjector()) {
      EXPECT_TRUE(lifted);
      if (ejected) {
        const double ejector_dx =
            IndexMotor::kEjectorMovementVelocity * time_left;
        position_ = ::std::min(position_ + ejector_dx,
                               IndexMotor::kEjectorStopPosition);
        EXPECT_FALSE(clamped);
      }
      printf("Ejector: Disc is at %f\n", position_);
    } else if (position_ == IndexMotor::kEjectorStopPosition) {
      printf("Shot: Disc is at %f\n", position_);
      has_been_shot_ = true;
    }
  }

  // Updates the position of the frisbee in the frisbee path.
  void UpdatePosition(double transfer_roller_position,
                      double index_roller_position,
                      bool clamped,
                      bool lifted,
                      bool ejected) {
    const double transfer_roller_velocity =
      (transfer_roller_position - transfer_roller_position_) / 0.01;
    const double index_roller_velocity =
      (index_roller_position - index_roller_position_) / 0.01;
    UpdatePositionForTime(transfer_roller_velocity,
                          index_roller_velocity,
                          clamped,
                          lifted,
                          ejected,
                          0.01);
    transfer_roller_position_ = transfer_roller_position;
    index_roller_position_ = index_roller_position;
  }

  // Returns if the disc has been shot and can be removed from the robot.
  bool has_been_shot() const {
    return has_been_shot_;
  }

  // Returns the position of the disc in the system.
  double position() const {
    return position_;
  }

  // Sets whether or not we have counted the delayed negedge.
  void set_counted_negedge_wait(bool counted_negedge_wait) {
    counted_negedge_wait_ = counted_negedge_wait;
  }

  // Returns if we have counted the delayed negedge.
  bool counted_negedge_wait() { return counted_negedge_wait_; }

  // Returns true if the negedge wait position is valid.
  bool has_bottom_disc_negedge_wait_position() {
    return has_bottom_disc_negedge_wait_position_;
  }

  // Returns the negedge wait position.
  double bottom_disc_negedge_wait_position() {
    return bottom_disc_negedge_wait_position_;
  }

  // Returns the last position where a posedge was seen.
  double top_disc_posedge_position() { return top_disc_posedge_position_; }

  // True if the top disc has seen a posedge.
  // Reading this flag clears it.
  bool has_top_disc_posedge_position() {
    bool prev = has_top_disc_posedge_position_;
    has_top_disc_posedge_position_ = false;
    return prev;
  }

  // Simulates the index roller moving without the disc moving.
  void OffsetIndex(double offset) {
    index_roller_position_ += offset;
  }

 private:
  // Previous transfer roller position for computing deltas.
  double transfer_roller_position_;
  // Previous index roller position for computing deltas.
  double index_roller_position_;
  // Position in the robot.
  double position_;
  // True if the disc has been shot.
  bool has_been_shot_;
  // True if the delay after the negedge of the beam break has occured.
  bool has_bottom_disc_negedge_wait_position_;
  // Posiiton of the indexer when the delayed negedge occures.
  double bottom_disc_negedge_wait_position_;
  // Time left after the negedge before we need to sample the indexer position.
  double after_negedge_time_left_;
  // Bool for the user to record if they have counted the negedge from this
  // disc.
  bool counted_negedge_wait_;
  // True if the top disc sensor posedge has occured and
  // hasn't been counted yet.
  bool has_top_disc_posedge_position_;
  // The position at which the posedge occured.
  double top_disc_posedge_position_;
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
        bottom_disc_posedge_count_(0),
        bottom_disc_negedge_count_(0),
        bottom_disc_negedge_wait_count_(0),
        bottom_disc_negedge_wait_position_(0),
        top_disc_posedge_count_(0),
        top_disc_posedge_position_(0.0),
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
    for (auto frisbee = frisbees.begin();
         frisbee != frisbees.end(); ++frisbee) {
      bottom_disc_detect |= frisbee->bottom_disc_detect();
    }
    return bottom_disc_detect;
  }

  // Returns true if the top disc sensor is triggered.
  bool TopDiscDetect() const {
    bool top_disc_detect = false;
    for (auto frisbee = frisbees.begin();
         frisbee != frisbees.end(); ++frisbee) {
      top_disc_detect |= frisbee->top_disc_detect();
    }
    return top_disc_detect;
  }

  // Updates all discs, and verifies that the state of the system is sane.
  void UpdateDiscs(bool clamped, bool lifted, bool ejected) {
    for (auto frisbee = frisbees.begin();
         frisbee != frisbees.end(); ++frisbee) {
      const bool old_bottom_disc_detect = frisbee->bottom_disc_detect();
      frisbee->UpdatePosition(transfer_roller_position(),
                              index_roller_position(),
                              clamped,
                              lifted,
                              ejected);

      // Look for disc detect edges and report them.
      const bool bottom_disc_detect = frisbee->bottom_disc_detect();
      if (old_bottom_disc_detect && !bottom_disc_detect) {
        printf("Negedge of disc\n");
        ++bottom_disc_negedge_count_;
      }

      if (!old_bottom_disc_detect && frisbee->bottom_disc_detect()) {
        printf("Posedge of disc\n");
        ++bottom_disc_posedge_count_;
      }

      // See if the frisbee has a delayed negedge and encoder value to report
      // back.
      if (frisbee->has_bottom_disc_negedge_wait_position()) {
        if (!frisbee->counted_negedge_wait()) {
          bottom_disc_negedge_wait_position_ =
              frisbee->bottom_disc_negedge_wait_position();
          ++bottom_disc_negedge_wait_count_;
          frisbee->set_counted_negedge_wait(true);
        }
      }
      if (frisbee->has_top_disc_posedge_position()) {
        ++top_disc_posedge_count_;
        top_disc_posedge_position_ = frisbee->top_disc_posedge_position();
      }
    }

    // Make sure nobody is too close to anybody else.
    Frisbee *last_frisbee = NULL;
    for (auto frisbee = frisbees.begin();
         frisbee != frisbees.end(); ++frisbee) {
      if (last_frisbee) {
        const double distance = frisbee->position() - last_frisbee->position();
        double min_distance;
        if (frisbee->IsTouchingTransfer() ||
            last_frisbee->IsTouchingTransfer()) {
          min_distance = 0.3;
        } else {
          min_distance =
              IndexMotor::ConvertDiscAngleToDiscPosition(M_PI * 2.0 / 3.0);
        }

        EXPECT_LT(min_distance, ::std::abs(distance)) << "Discs too close";
      }
      last_frisbee = &*frisbee;
    }

    // Remove any shot frisbees.
    for (int i = 0; i < static_cast<int>(frisbees.size()); ++i) {
      if (frisbees[i].has_been_shot()) {
        shot_frisbees.push_back(frisbees[i]);
        frisbees.erase(frisbees.begin() + i);
        --i;
      }
    }
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::IndexLoop::Position> position =
        my_index_loop_.position.MakeMessage();
    position->index_position = index_roller_position();
    position->bottom_disc_detect = BottomDiscDetect();
    position->top_disc_detect = TopDiscDetect();
    position->bottom_disc_posedge_count = bottom_disc_posedge_count_;
    position->bottom_disc_negedge_count = bottom_disc_negedge_count_;
    position->bottom_disc_negedge_wait_count = bottom_disc_negedge_wait_count_;
    position->bottom_disc_negedge_wait_position =
        bottom_disc_negedge_wait_position_;
    position->top_disc_posedge_count = top_disc_posedge_count_;
    position->top_disc_posedge_position = top_disc_posedge_position_;
    printf("bdd: %x tdd: %x posedge %d negedge %d "
           "delaycount %d delaypos %f topcount %d toppos %f\n",
           position->bottom_disc_detect,
           position->top_disc_detect,
           position->bottom_disc_posedge_count,
           position->bottom_disc_negedge_count,
           position->bottom_disc_negedge_wait_count,
           position->bottom_disc_negedge_wait_position,
           position->top_disc_posedge_count,
           position->top_disc_posedge_position);
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

    UpdateDiscs(my_index_loop_.output->disc_clamped,
                my_index_loop_.output->loader_up,
                my_index_loop_.output->disc_ejected);
  }

  // Simulates the index roller moving without the disc moving.
  void OffsetIndices(double offset) {
    for (auto frisbee = frisbees.begin();
         frisbee != frisbees.end(); ++frisbee) {
      frisbee->OffsetIndex(offset);
    }
  }

  // Plants for the index and transfer rollers.
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> index_plant_;
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> transfer_plant_;

  // Posedge and negedge counts for the beam break.
  int32_t bottom_disc_posedge_count_;
  int32_t bottom_disc_negedge_count_;

  // Delayed negedge count and corrisponding position.
  int32_t bottom_disc_negedge_wait_count_;
  int32_t bottom_disc_negedge_wait_position_;

  // Posedge count and position for the upper disc sensor.
  int32_t top_disc_posedge_count_;
  double top_disc_posedge_position_;

  // Returns the absolute angle of the index.
  double index_roller_position() const {
    return index_plant_->Y(0, 0);
  }

  // Returns the absolute angle of the index.
  double transfer_roller_position() const {
    return transfer_plant_->Y(0, 0);
  }

  // Frisbees being tracked in the robot.
  ::std::vector<Frisbee> frisbees;
  // Frisbees that have been shot.
  ::std::vector<Frisbee> shot_frisbees;

 private:
  // Control loop for the indexer.
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

  // Lets N cycles of time pass.
  void SimulateNCycles(int cycles) {
    for (int i = 0; i < cycles; ++i) {
      index_motor_plant_.SendPositionMessage();
      index_motor_.Iterate();
      index_motor_plant_.Simulate();
      SendDSPacket(true);
      UpdateTime();
    }
  }

  // Loads n discs into the indexer at the bottom.
  void LoadNDiscs(int n) {
    my_index_loop_.goal.MakeWithBuilder().goal_state(2).Send();
    // Spin it up.
    SimulateNCycles(100);

    EXPECT_EQ(0, index_motor_plant_.index_roller_position());
    my_index_loop_.status.FetchLatest();
    EXPECT_TRUE(my_index_loop_.status->ready_to_intake);

    // Stuff N discs in, waiting between each one for a tiny bit of time so they
    // don't get too close.
    int num_grabbed = 0;
    int wait_counter = 0;
    while (num_grabbed < n) {
      index_motor_plant_.SendPositionMessage();
      index_motor_.Iterate();
      if (!index_motor_plant_.BottomDiscDetect()) {
        if (wait_counter > 0) {
          --wait_counter;
        } else {
          index_motor_plant_.InsertDisc();
          ++num_grabbed;
          wait_counter = 5;
        }
      }
      index_motor_plant_.Simulate();
      SendDSPacket(true);
      UpdateTime();
    }

    // Settle.
    for (int i = 0; i < 100; ++i) {
      index_motor_plant_.SendPositionMessage();
      index_motor_.Iterate();
      index_motor_plant_.Simulate();
      SendDSPacket(true);
      UpdateTime();
    }
  }

  // Loads 2 discs, and then offsets them.  We then send the first disc to the
  // grabber, and the second disc back down to the bottom.  Verify that both
  // discs get found correctly.  Positive numbers shift the discs up.
  void TestDualLostDiscs(double top_disc_offset, double bottom_disc_offset) {
    LoadNDiscs(2);

    // Move them in the indexer so they need to be re-found.
    // The top one is moved further than the bottom one so that both edges need to
    // be inspected.
    index_motor_plant_.frisbees[0].OffsetIndex(
         IndexMotor::ConvertDiscPositionToIndex(top_disc_offset));
    index_motor_plant_.frisbees[1].OffsetIndex(
         IndexMotor::ConvertDiscPositionToIndex(bottom_disc_offset));

    // Lift the discs up to the top.  Wait a while to let the system settle and
    // verify that they don't collide.
    my_index_loop_.goal.MakeWithBuilder().goal_state(3).Send();
    SimulateNCycles(300);

    // Verify that the disc has been grabbed.
    my_index_loop_.output.FetchLatest();
    EXPECT_TRUE(my_index_loop_.output->disc_clamped);
    // And that we are preloaded.
    my_index_loop_.status.FetchLatest();
    EXPECT_TRUE(my_index_loop_.status->preloaded);

    // Pull the disc back down.
    my_index_loop_.goal.MakeWithBuilder().goal_state(2).Send();
    SimulateNCycles(300);

    EXPECT_NEAR(IndexMotor::kReadyToLiftPosition,
        index_motor_plant_.frisbees[0].position(), 0.01);
    EXPECT_NEAR(
        (IndexMotor::kIndexStartPosition +
         IndexMotor::ConvertDiscAngleToDiscPosition(M_PI)),
        index_motor_plant_.frisbees[1].position(), 0.02);

    // Verify that we found the disc as accurately as the FPGA allows.
    my_index_loop_.position.FetchLatest();
    EXPECT_NEAR(
        index_motor_.frisbees()[0].absolute_position(
            my_index_loop_.position->index_position),
        index_motor_plant_.frisbees[1].position(), 0.0001);
  }

  // Copy of core that works in this process only.
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  IndexLoop my_index_loop_;

  // Create a loop and simulation plant.
  IndexMotor index_motor_;
  IndexMotorSimulation index_motor_plant_;

  // Number of loop cycles that have been executed for tracking the current
  // time.
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
    if (i > 0) {
      EXPECT_TRUE(my_index_loop_.status.FetchLatest());
      EXPECT_TRUE(my_index_loop_.status->ready_to_intake);
    }
    index_motor_plant_.Simulate();
    SendDSPacket(true);
    UpdateTime();
  }

  my_index_loop_.status.FetchLatest();
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 1);
  EXPECT_EQ(static_cast<size_t>(1), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(
      IndexMotor::kIndexStartPosition + IndexMotor::ConvertDiscAngleToDiscPosition(M_PI),
      index_motor_plant_.frisbees[0].position(), 0.05);
}

// Tests that the index grabs 1 disc and places it at the correct position when
// told to hold immediately after the disc starts into the bot.
TEST_F(IndexTest, GrabAndHold) {
  my_index_loop_.goal.MakeWithBuilder().goal_state(2).Send();
  for (int i = 0; i < 200; ++i) {
    index_motor_plant_.SendPositionMessage();
    index_motor_.Iterate();
    if (i == 100) {
      EXPECT_EQ(0, index_motor_plant_.index_roller_position());
      index_motor_plant_.InsertDisc();
    } else if (i == 102) {
      // The disc has been seen.  Tell the indexer to now hold.
      my_index_loop_.goal.MakeWithBuilder().goal_state(0).Send();
    } else if (i > 102) {
      my_index_loop_.status.FetchLatest();
      EXPECT_FALSE(my_index_loop_.status->ready_to_intake);
    }
    index_motor_plant_.Simulate();
    SendDSPacket(true);
    UpdateTime();
  }

  my_index_loop_.status.FetchLatest();
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 1);
  EXPECT_EQ(static_cast<size_t>(1), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(
      (IndexMotor::kIndexStartPosition +
       IndexMotor::ConvertDiscAngleToDiscPosition(M_PI)),
      index_motor_plant_.frisbees[0].position(), 0.04);
}

// Tests that the index grabs two discs and places them at the correct
// positions.
TEST_F(IndexTest, GrabTwoDiscs) {
  LoadNDiscs(2);

  EXPECT_TRUE(my_index_loop_.status.FetchLatest());
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 2);
  EXPECT_EQ(static_cast<size_t>(2), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(
      (IndexMotor::kIndexStartPosition +
       IndexMotor::ConvertDiscAngleToDiscPosition(M_PI)),
      index_motor_plant_.frisbees[1].position(), 0.10);
  EXPECT_NEAR(
      IndexMotor::ConvertDiscAngleToDiscPosition(M_PI),
      (index_motor_plant_.frisbees[0].position() -
       index_motor_plant_.frisbees[1].position()), 0.10);
}

// Tests that the index grabs 2 discs, and loads one up into the loader to get
// ready to shoot.  It then pulls the second disc back down to be ready to
// intake more.
TEST_F(IndexTest, ReadyGrabsOneDisc) {
  LoadNDiscs(2);

  // Lift the discs up to the top.  Wait a while to let the system settle and
  // verify that they don't collide.
  my_index_loop_.goal.MakeWithBuilder().goal_state(3).Send();
  SimulateNCycles(300);

  // Verify that the disc has been grabbed.
  my_index_loop_.output.FetchLatest();
  EXPECT_TRUE(my_index_loop_.output->disc_clamped);
  // And that we are preloaded.
  my_index_loop_.status.FetchLatest();
  EXPECT_TRUE(my_index_loop_.status->preloaded);

  // Pull the disc back down and verify that the transfer roller doesn't turn on
  // until we are ready.
  my_index_loop_.goal.MakeWithBuilder().goal_state(2).Send();
  for (int i = 0; i < 100; ++i) {
    index_motor_plant_.SendPositionMessage();
    index_motor_.Iterate();

    EXPECT_TRUE(my_index_loop_.status.FetchLatest());
    EXPECT_TRUE(my_index_loop_.output.FetchLatest());
    if (!my_index_loop_.status->ready_to_intake) {
      EXPECT_EQ(my_index_loop_.output->transfer_voltage, 0)
          << "Transfer should be off until indexer is ready";
    }

    index_motor_plant_.Simulate();
    SendDSPacket(true);
    UpdateTime();
  }

  my_index_loop_.status.FetchLatest();
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 1);
  EXPECT_EQ(my_index_loop_.status->total_disc_count, 2);
  my_index_loop_.output.FetchLatest();
  EXPECT_TRUE(my_index_loop_.output->disc_clamped);

  EXPECT_EQ(static_cast<size_t>(2), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(IndexMotor::kReadyToLiftPosition,
      index_motor_plant_.frisbees[0].position(), 0.01);
  printf("Top disc error is %f\n",
         IndexMotor::kReadyToLiftPosition -
         index_motor_plant_.frisbees[0].position());
  EXPECT_NEAR(
      (IndexMotor::kIndexStartPosition +
       IndexMotor::ConvertDiscAngleToDiscPosition(M_PI)),
      index_motor_plant_.frisbees[1].position(), 0.02);
  printf("Bottom disc error is %f\n",
      (IndexMotor::kIndexStartPosition +
       IndexMotor::ConvertDiscAngleToDiscPosition(M_PI))-
      index_motor_plant_.frisbees[1].position());
}

// Tests that the index grabs 1 disc and continues to pull it in correctly when
// in the READY_LOWER state.  The transfer roller should be disabled then.
TEST_F(IndexTest, GrabAndReady) {
  my_index_loop_.goal.MakeWithBuilder().goal_state(2).Send();
  for (int i = 0; i < 200; ++i) {
    index_motor_plant_.SendPositionMessage();
    index_motor_.Iterate();
    if (i == 100) {
      EXPECT_EQ(0, index_motor_plant_.index_roller_position());
      index_motor_plant_.InsertDisc();
    } else if (i == 102) {
      my_index_loop_.goal.MakeWithBuilder().goal_state(1).Send();
    } else if (i > 150) {
      my_index_loop_.status.FetchLatest();
      EXPECT_TRUE(my_index_loop_.status->ready_to_intake);
      my_index_loop_.output.FetchLatest();
      EXPECT_EQ(my_index_loop_.output->transfer_voltage, 0.0);
    }
    index_motor_plant_.Simulate();
    SendDSPacket(true);
    UpdateTime();
  }

  my_index_loop_.status.FetchLatest();
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 1);
  EXPECT_EQ(static_cast<size_t>(1), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(
      (IndexMotor::kIndexStartPosition +
       IndexMotor::ConvertDiscAngleToDiscPosition(M_PI)),
      index_motor_plant_.frisbees[0].position(), 0.04);
}

// Tests that grabbing 4 discs ends up with 4 discs in the bot and us no longer
// ready.
TEST_F(IndexTest, GrabFourDiscs) {
  LoadNDiscs(4);

  EXPECT_TRUE(my_index_loop_.output.FetchLatest());
  EXPECT_EQ(my_index_loop_.output->transfer_voltage, 0.0);
  EXPECT_TRUE(my_index_loop_.status.FetchLatest());
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 4);
  EXPECT_FALSE(my_index_loop_.status->ready_to_intake);
  EXPECT_EQ(static_cast<size_t>(4), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(
      IndexMotor::kIndexStartPosition + IndexMotor::ConvertDiscAngleToDiscPosition(M_PI),
      index_motor_plant_.frisbees[3].position(), 0.10);
  EXPECT_NEAR(
      IndexMotor::ConvertDiscAngleToDiscPosition(M_PI),
      (index_motor_plant_.frisbees[0].position() -
       index_motor_plant_.frisbees[1].position()), 0.10);
  EXPECT_NEAR(
      IndexMotor::ConvertDiscAngleToDiscPosition(M_PI),
      (index_motor_plant_.frisbees[1].position() -
       index_motor_plant_.frisbees[2].position()), 0.10);
  EXPECT_NEAR(
      IndexMotor::ConvertDiscAngleToDiscPosition(M_PI),
      (index_motor_plant_.frisbees[2].position() -
       index_motor_plant_.frisbees[3].position()), 0.10);
}

// Tests that shooting 4 discs works.
TEST_F(IndexTest, ShootFourDiscs) {
  LoadNDiscs(4);

  EXPECT_EQ(static_cast<size_t>(4), index_motor_plant_.frisbees.size());

  my_index_loop_.goal.MakeWithBuilder().goal_state(4).Send();

  // Lifting and shooting takes a while...
  SimulateNCycles(300);

  my_index_loop_.status.FetchLatest();
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 0);
  EXPECT_EQ(my_index_loop_.status->total_disc_count, 4);
  my_index_loop_.output.FetchLatest();
  EXPECT_FALSE(my_index_loop_.output->disc_clamped);
  EXPECT_FALSE(my_index_loop_.output->loader_up);
  EXPECT_FALSE(my_index_loop_.output->disc_ejected);

  EXPECT_EQ(static_cast<size_t>(4), index_motor_plant_.shot_frisbees.size());
}

// Tests that discs aren't pulled out of the loader half way through being
// grabbed when being asked to index.
TEST_F(IndexTest, PreloadToIndexEarlyTransition) {
  LoadNDiscs(2);

  // Lift the discs up to the top.  Wait a while to let the system settle and
  // verify that they don't collide.
  my_index_loop_.goal.MakeWithBuilder().goal_state(3).Send();
  for (int i = 0; i < 300; ++i) {
    index_motor_plant_.SendPositionMessage();
    index_motor_.Iterate();
    index_motor_plant_.Simulate();
    SendDSPacket(true);
    UpdateTime();
    // Drop out of the loop as soon as it enters the loader.
    // This will require it to finish the job before intaking more.
    my_index_loop_.status.FetchLatest();
    if (index_motor_plant_.frisbees[0].position() >
        IndexMotor::kLoaderFreeStopPosition) {
      break;
    }
  }

  // Pull the disc back down and verify that the transfer roller doesn't turn on
  // until we are ready.
  my_index_loop_.goal.MakeWithBuilder().goal_state(1).Send();
  SimulateNCycles(100);

  my_index_loop_.status.FetchLatest();
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 1);
  EXPECT_EQ(my_index_loop_.status->total_disc_count, 2);
  my_index_loop_.output.FetchLatest();
  EXPECT_TRUE(my_index_loop_.output->disc_clamped);

  EXPECT_EQ(static_cast<size_t>(2), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(IndexMotor::kReadyToLiftPosition,
      index_motor_plant_.frisbees[0].position(), 0.01);
  EXPECT_NEAR(
      (IndexMotor::kIndexStartPosition +
       IndexMotor::ConvertDiscAngleToDiscPosition(M_PI)),
      index_motor_plant_.frisbees[1].position(), 0.10);
}

// Tests that disabling while grabbing a disc doesn't cause problems.
TEST_F(IndexTest, HandleDisable) {
  my_index_loop_.goal.MakeWithBuilder().goal_state(2).Send();
  for (int i = 0; i < 200; ++i) {
    index_motor_plant_.SendPositionMessage();
    index_motor_.Iterate();
    if (i == 100) {
      EXPECT_EQ(0, index_motor_plant_.index_roller_position());
      index_motor_plant_.InsertDisc();
    } else if (i == 102) {
      my_index_loop_.goal.MakeWithBuilder().goal_state(1).Send();
    } else if (i > 150) {
      my_index_loop_.status.FetchLatest();
      EXPECT_TRUE(my_index_loop_.status->ready_to_intake);
      my_index_loop_.output.FetchLatest();
      EXPECT_EQ(my_index_loop_.output->transfer_voltage, 0.0);
    }
    index_motor_plant_.Simulate();
    SendDSPacket(i < 102 || i > 110);
    UpdateTime();
  }

  my_index_loop_.status.FetchLatest();
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 1);
  EXPECT_EQ(static_cast<size_t>(1), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(
      (IndexMotor::kIndexStartPosition +
       IndexMotor::ConvertDiscAngleToDiscPosition(M_PI)),
      index_motor_plant_.frisbees[0].position(), 0.04);
}

// Tests that we can shoot after grabbing in the loader.
TEST_F(IndexTest, GrabbedToShoot) {
  LoadNDiscs(2);

  // Lift the discs up to the top.  Wait a while to let the system settle and
  // verify that they don't collide.
  my_index_loop_.goal.MakeWithBuilder().goal_state(3).Send();
  SimulateNCycles(300);

  // Verify that it is preloaded.
  my_index_loop_.status.FetchLatest();
  EXPECT_TRUE(my_index_loop_.status->preloaded);

  // Shoot them all.
  my_index_loop_.goal.MakeWithBuilder().goal_state(4).Send();
  SimulateNCycles(200);

  my_index_loop_.status.FetchLatest();
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 0);
  EXPECT_EQ(my_index_loop_.status->total_disc_count, 2);
  EXPECT_FALSE(my_index_loop_.status->preloaded);
}

// Tests that the cRIO can reboot and we don't loose discs.
TEST_F(IndexTest, cRIOReboot) {
  LoadNDiscs(2);

  SimulateNCycles(100);
  for (int i = 0; i < 100; ++i) {
    // No position for a while is a cRIO reboot.
    index_motor_.Iterate();
    index_motor_plant_.Simulate();
    SendDSPacket(false);
    UpdateTime();
  }

  // Shift the plant.
  const double kPlantOffset = 5000.0;
  index_motor_plant_.index_plant_->Y(0, 0) += kPlantOffset;
  index_motor_plant_.index_plant_->X(0, 0) += kPlantOffset;
  index_motor_plant_.bottom_disc_posedge_count_ = 971;
  index_motor_plant_.bottom_disc_negedge_count_ = 971;
  index_motor_plant_.bottom_disc_negedge_wait_count_ = 971;
  index_motor_plant_.bottom_disc_negedge_wait_position_ = -1502;

  // Shift the discs
  index_motor_plant_.OffsetIndices(kPlantOffset);
  // Let time elapse to see if the loop wants to move the discs or not.
  SimulateNCycles(1000);

  // Verify that 2 discs are at the bottom of the hopper.
  EXPECT_TRUE(my_index_loop_.status.FetchLatest());
  EXPECT_EQ(my_index_loop_.status->hopper_disc_count, 2);
  EXPECT_EQ(static_cast<size_t>(2), index_motor_plant_.frisbees.size());
  EXPECT_NEAR(
      (IndexMotor::kIndexStartPosition +
       IndexMotor::ConvertDiscAngleToDiscPosition(M_PI)),
      index_motor_plant_.frisbees[1].position(), 0.10);
  EXPECT_NEAR(
      IndexMotor::ConvertDiscAngleToDiscPosition(M_PI),
      (index_motor_plant_.frisbees[0].position() -
       index_motor_plant_.frisbees[1].position()), 0.10);
}

// Tests that invalid states are rejected.
TEST_F(IndexTest, InvalidStateTest) {
  my_index_loop_.goal.MakeWithBuilder().goal_state(10).Send();
  SimulateNCycles(2);
  // Verify that the goal is correct.
  EXPECT_GE(4, static_cast<int>(index_motor_.safe_goal_));
  EXPECT_LE(0, static_cast<int>(index_motor_.safe_goal_));
}

// Tests that the motor is turned off after a number of cycles of low power.
TEST_F(IndexTest, ZeroPowerAfterTimeout) {
  LoadNDiscs(4);
  SimulateNCycles(100);

  // Verify that the motor is hard off.  This relies on floating point math
  // never really getting to 0 unless you set it explicitly.
  my_index_loop_.output.FetchLatest();
  EXPECT_EQ(my_index_loop_.output->index_voltage, 0.0);
}

// Tests that preloading 2 discs relocates the discs if they shift on the
// indexer.  Test shifting all 4 ways.
TEST_F(IndexTest, ShiftedDiscsAreRefound) {
  TestDualLostDiscs(0.10, 0.15);
}

TEST_F(IndexTest, ShiftedDiscsAreRefoundOtherSeperation) {
  TestDualLostDiscs(0.15, 0.10);
}

TEST_F(IndexTest, ShiftedDownDiscsAreRefound) {
  TestDualLostDiscs(-0.15, -0.10);
}

TEST_F(IndexTest, ShiftedDownDiscsAreRefoundOtherSeperation) {
  TestDualLostDiscs(-0.10, -0.15);
}

// TODO(aschuh): Test that we find discs corectly when moving them up.

// TODO(aschuh): Exercise the disc coming down from above code and verify it.
// If possible.


}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
