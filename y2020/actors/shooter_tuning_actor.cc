#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <random>
#include <sstream>
#include <utility>

#include "aos/init.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "glog/logging.h"
#include "y2020/constants.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/control_loops/superstructure/shooter/shooter_tuning_params_generated.h"
#include "y2020/control_loops/superstructure/shooter/shooter_tuning_readings_generated.h"
#include "y2020/control_loops/superstructure/superstructure_goal_generated.h"

namespace y2020::actors {

namespace superstructure = y2020::control_loops::superstructure;

// Actor for tuning the shooter flywheel velocities.
// We found that the shooter is most precise when the ball velocity varies the
// least. To figure out which finisher and accelerator velocities cause the ball
// velocity to vary the least, this does a sweep of a range of those velocities
// in randomized order, and shoots balls at each set of velocities. It also
// fetches the ball velocity at each iteration, which we will use to calculate
// each iteration's deviation in ball velocity and figure out which flywheel
// velocities are the best with the csv of velocities that this outputs.
class ShooterTuningActor : public frc971::autonomous::BaseAutonomousActor {
 public:
  explicit ShooterTuningActor(aos::EventLoop *event_loop);

  bool RunAction(
      const frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void SendSuperstructureGoal();
  void WaitAndWriteBallData();

  aos::Sender<superstructure::Goal> superstructure_goal_sender_;
  aos::Fetcher<superstructure::shooter::TuningReadings>
      tuning_readings_fetcher_;
  const superstructure::shooter::TuningParams *tuning_params_;

  // Vector of (velocity_accelerator, velocity finisher) containing all velocity
  // values in the sweep in randomized order.
  std::vector<std::pair<double, double>> velocities_;

  bool shooting_ = false;
  double velocity_finisher_ = 0.0;
  double velocity_accelerator_ = 0.0;

  std::ofstream fout_;
};

ShooterTuningActor::ShooterTuningActor(aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()),
      superstructure_goal_sender_(
          event_loop->MakeSender<superstructure::Goal>("/superstructure")),
      tuning_readings_fetcher_(
          event_loop->MakeFetcher<superstructure::shooter::TuningReadings>(
              "/superstructure")) {
  auto tuning_params_fetcher =
      event_loop->MakeFetcher<superstructure::shooter::TuningParams>(
          "/superstructure");
  CHECK(tuning_params_fetcher.Fetch());
  tuning_params_ = tuning_params_fetcher.get();
  const auto finisher = tuning_params_->finisher();
  const auto accelerator = tuning_params_->accelerator();

  LOG(INFO) << "Tuning with fininisher from " << finisher->velocity_initial()
            << " to " << finisher->velocity_final() << " by "
            << finisher->velocity_increment();
  LOG(INFO) << "Tuning with accelerator from "
            << accelerator->velocity_initial() << " to "
            << accelerator->velocity_final() << " by "
            << accelerator->velocity_increment();
  // Add the velocities for the sweep
  for (velocity_finisher_ = finisher->velocity_initial();
       velocity_finisher_ <= finisher->velocity_final();
       velocity_finisher_ += finisher->velocity_increment()) {
    for (velocity_accelerator_ = accelerator->velocity_initial();
         velocity_accelerator_ <= accelerator->velocity_final();
         velocity_accelerator_ += accelerator->velocity_increment()) {
      for (int i = 0; i < tuning_params_->balls_per_iteration(); i++) {
        velocities_.emplace_back(velocity_accelerator_, velocity_finisher_);
      }
    }
  }
  // Randomize the ordering of the velocities
  std::srand(std::time(nullptr));
  std::random_device random_device;
  std::mt19937 generator(random_device());
  std::shuffle(velocities_.begin(), velocities_.end(), generator);
}

bool ShooterTuningActor::RunAction(
    const frc971::autonomous::AutonomousActionParams * /*params*/) {
  fout_.open("shooter_tuning_data.csv", std::ios_base::app);
  LOG(INFO) << "velocity_accelerator,velocity_finisher,velocity_ball";

  shooting_ = true;
  for (size_t i = 0; i < velocities_.size(); i++) {
    LOG(INFO) << "Shooting ball " << (i + 1) << " out of "
              << velocities_.size();
    velocity_accelerator_ = velocities_[i].first;
    velocity_finisher_ = velocities_[i].second;
    SendSuperstructureGoal();
    WaitAndWriteBallData();

    if (i % 100 == 0 && i != 0) {
      LOG(INFO) << "Pausing to cool for 2 minutes";
      shooting_ = false;
      velocity_accelerator_ = 0.0;
      velocity_finisher_ = 0.0;
      SendSuperstructureGoal();
      std::this_thread::sleep_for(std::chrono::seconds(120));
      shooting_ = true;
    }
  }

  shooting_ = false;
  velocity_finisher_ = 0.0;
  velocity_accelerator_ = 0.0;
  SendSuperstructureGoal();
  fout_.close();
  LOG(INFO) << "Done!";
  return true;
}

void ShooterTuningActor::WaitAndWriteBallData() {
  aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                    event_loop()->monotonic_now(),
                                    frc971::controls::kLoopFrequency / 2);
  bool ball_detected = false;

  while (!ball_detected) {
    phased_loop.SleepUntilNext();

    if (tuning_readings_fetcher_.FetchNext()) {
      ball_detected = true;
      std::stringstream output;
      output << velocity_accelerator_ << ',' << velocity_finisher_ << ','
             << tuning_readings_fetcher_->velocity_ball() << std::endl;
      const auto output_str = output.str();
      LOG(INFO) << output_str;
      fout_ << output_str;
    }
  }
}

void ShooterTuningActor::SendSuperstructureGoal() {
  auto builder = superstructure_goal_sender_.MakeBuilder();

  auto shooter_builder = builder.MakeBuilder<superstructure::ShooterGoal>();
  shooter_builder.add_velocity_finisher(velocity_finisher_);
  shooter_builder.add_velocity_accelerator(velocity_accelerator_);
  auto shooter_offset = shooter_builder.Finish();

  superstructure::Goal::Builder superstructure_builder =
      builder.MakeBuilder<superstructure::Goal>();

  superstructure_builder.add_shooter(shooter_offset);
  superstructure_builder.add_shooting(shooting_);

  if (builder.Send(superstructure_builder.Finish()) !=
      aos::RawSender::Error::kOk) {
    AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
  }
}

}  // namespace y2020::actors

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  aos::ShmEventLoop event_loop(&config.message());
  y2020::constants::InitValues();
  y2020::actors::ShooterTuningActor actor(&event_loop);

  event_loop.Run();

  return 0;
}
