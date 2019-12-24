#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <chrono>
#include <thread>
#include <mutex>
#include <functional>

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Compressor.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/PowerDistributionPanel.h"
#include "frc971/wpilib/ahal/Relay.h"
#include "frc971/wpilib/ahal/Talon.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#undef ERROR

#include "aos/controls/control_loops_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/make_unique.h"
#include "aos/robot_state/robot_state_generated.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/interrupt_edge_counting.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/sensor_reader.h"
#include "y2012/control_loops/accessories/accessories_generated.h"

namespace accessories = ::y2012::control_loops::accessories;
using namespace frc;
using aos::make_unique;

namespace y2012 {
namespace wpilib {

double drivetrain_translate(int32_t in) {
  return -static_cast<double>(in) /
         (256.0 /*cpr*/ * 4.0 /*4x*/) *
         1 *
         (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI) * 2.0 / 2.0;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         1 *
         (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI) * 2.0 / 2.0;
}

class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop)
      : ::frc971::wpilib::SensorReader(event_loop),
        accessories_position_sender_(
            event_loop->MakeSender<::aos::control_loops::Position>(
                "/accessories")),
        drivetrain_position_sender_(
            event_loop
                ->MakeSender<::frc971::control_loops::drivetrain::Position>(
                    "/drivetrain")) {}

  void RunIteration() {
    {
      auto builder = drivetrain_position_sender_.MakeBuilder();

      frc971::control_loops::drivetrain::Position::Builder position_builder =
          builder.MakeBuilder<frc971::control_loops::drivetrain::Position>();
      position_builder.add_right_encoder(
          drivetrain_translate(drivetrain_right_encoder_->GetRaw()));
      position_builder.add_left_encoder(
          -drivetrain_translate(drivetrain_left_encoder_->GetRaw()));
      position_builder.add_left_speed(
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod()));
      position_builder.add_right_speed(drivetrain_velocity_translate(
          drivetrain_right_encoder_->GetPeriod()));

      builder.Send(position_builder.Finish());
    }

    {
      auto builder = accessories_position_sender_.MakeBuilder();
      builder.Send(
          builder.MakeBuilder<::aos::control_loops::Position>().Finish());
    }
  }

 private:
  ::aos::Sender<::aos::control_loops::Position> accessories_position_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(::aos::ShmEventLoop *event_loop,
                 const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : event_loop_(event_loop),
        pcm_(pcm),
        drivetrain_fetcher_(
            event_loop_
                ->MakeFetcher<::frc971::control_loops::drivetrain::Output>(
                    "/drivetrain")),
        accessories_fetcher_(
            event_loop_
                ->MakeFetcher<::y2012::control_loops::accessories::Message>(
                    "/accessories")),
        pneumatics_to_log_sender_(
            event_loop->MakeSender<::frc971::wpilib::PneumaticsToLog>("/aos")) {
    event_loop->set_name("Solenoids");
    event_loop_->SetRuntimeRealtimePriority(27);

    event_loop_->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                               ::std::chrono::milliseconds(20),
                               ::std::chrono::milliseconds(1));
  }

  void set_drivetrain_high(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    drivetrain_high_ = ::std::move(s);
  }

  void set_drivetrain_low(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    drivetrain_low_ = ::std::move(s);
  }

  void set_s1(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    s1_ = ::std::move(s);
  }

  void set_s2(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    s2_ = ::std::move(s);
  }

  void set_s3(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    s3_ = ::std::move(s);
  }

  void Loop(const int iterations) {
    if (iterations != 1) {
      AOS_LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
    }

    {
      accessories_fetcher_.Fetch();
      if (accessories_fetcher_.get()) {
        s1_->Set(accessories_fetcher_->solenoids()->Get(0));
        s2_->Set(accessories_fetcher_->solenoids()->Get(1));
        s3_->Set(accessories_fetcher_->solenoids()->Get(2));
      }
    }

    {
      drivetrain_fetcher_.Fetch();
      if (drivetrain_fetcher_.get()) {
        const bool high = drivetrain_fetcher_->left_high() ||
                          drivetrain_fetcher_->right_high();
        drivetrain_high_->Set(high);
        drivetrain_low_->Set(!high);
      }
    }

    {
      auto builder = pneumatics_to_log_sender_.MakeBuilder();

      ::frc971::wpilib::PneumaticsToLog::Builder to_log_builder =
          builder.MakeBuilder<frc971::wpilib::PneumaticsToLog>();
      pcm_->Flush();
      to_log_builder.add_read_solenoids(pcm_->GetAll());
      builder.Send(to_log_builder.Finish());
    }
  }

 private:
  ::aos::EventLoop *event_loop_;

  const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_high_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_low_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s1_, s2_, s3_;

  ::std::unique_ptr<Compressor> compressor_;

  ::aos::Fetcher<::frc971::control_loops::drivetrain::Output>
      drivetrain_fetcher_;
  ::aos::Fetcher<::y2012::control_loops::accessories::Message>
      accessories_fetcher_;

  aos::Sender<::frc971::wpilib::PneumaticsToLog> pneumatics_to_log_sender_;
};

class AccessoriesWriter : public ::frc971::wpilib::LoopOutputHandler<
                              control_loops::accessories::Message> {
 public:
  AccessoriesWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<
            control_loops::accessories::Message>(event_loop, "/accessories") {}

  void set_talon1(::std::unique_ptr<Talon> t) { talon1_ = ::std::move(t); }

  void set_talon2(::std::unique_ptr<Talon> t) { talon2_ = ::std::move(t); }

 private:
  virtual void Write(
      const control_loops::accessories::Message &output) override {
    talon1_->SetSpeed(output.sticks()->Get(0));
    talon2_->SetSpeed(output.sticks()->Get(1));
  }

  virtual void Stop() override {
    AOS_LOG(WARNING, "shooter output too old\n");
    talon1_->SetDisabled();
    talon2_->SetDisabled();
  }

  ::std::unique_ptr<Talon> talon1_, talon2_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<Encoder> make_encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
  }

  void Run() override {
    aos::FlatbufferDetachedBuffer<aos::Configuration> config =
        aos::configuration::ReadConfig("config.json");

    // Thread 1.
    ::aos::ShmEventLoop joystick_sender_event_loop(&config.message());
    ::frc971::wpilib::JoystickSender joystick_sender(
        &joystick_sender_event_loop);
    AddLoop(&joystick_sender_event_loop);

    // Thread 2.
    ::aos::ShmEventLoop sensor_reader_event_loop(&config.message());
    SensorReader sensor_reader(&sensor_reader_event_loop);
    sensor_reader.set_drivetrain_left_encoder(make_encoder(0));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(1));
    AddLoop(&sensor_reader_event_loop);

    // Thread 3.
    ::aos::ShmEventLoop output_event_loop(&config.message());
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<Talon>(new Talon(3)), true);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<Talon>(new Talon(4)), false);

    ::y2012::wpilib::AccessoriesWriter accessories_writer(&output_event_loop);
    accessories_writer.set_talon1(::std::unique_ptr<Talon>(new Talon(5)));
    accessories_writer.set_talon2(::std::unique_ptr<Talon>(new Talon(6)));
    AddLoop(&output_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop solenoid_writer_event_loop(&config.message());
    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(&solenoid_writer_event_loop, pcm);
    solenoid_writer.set_drivetrain_high(pcm->MakeSolenoid(0));
    solenoid_writer.set_drivetrain_low(pcm->MakeSolenoid(2));
    solenoid_writer.set_s1(pcm->MakeSolenoid(1));
    solenoid_writer.set_s2(pcm->MakeSolenoid(3));
    solenoid_writer.set_s3(pcm->MakeSolenoid(4));
    AddLoop(&solenoid_writer_event_loop);

    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2012


AOS_ROBOT_CLASS(::y2012::wpilib::WPILibRobot);
