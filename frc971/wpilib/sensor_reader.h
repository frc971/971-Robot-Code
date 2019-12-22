#ifndef FRC971_WPILIB_SENSOR_READER_H_
#define FRC971_WPILIB_SENSOR_READER_H_

#include <atomic>
#include <chrono>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/robot_state/robot_state_generated.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DigitalInput.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace frc971 {
namespace wpilib {

class SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop);
  virtual ~SensorReader() {}

  // Updates the fast and medium encoder filter frequencies.
  void UpdateFastEncoderFilterHz(int hz);
  void UpdateMediumEncoderFilterHz(int hz);

  // Sets the left drivetrain encoder.
  void set_drivetrain_left_encoder(::std::unique_ptr<frc::Encoder> encoder);

  // Sets the right drivetrain encoder.
  void set_drivetrain_right_encoder(::std::unique_ptr<frc::Encoder> encoder);

  // Adds a sensor to DMA.
  void AddToDMA(DMASampleHandlerInterface *handler) {
    if (!dma_synchronizer_) {
      dma_synchronizer_.reset(
          new ::frc971::wpilib::DMASynchronizer(std::make_unique<DMA>()));
    }
    dma_synchronizer_->Add(handler);
  }

  // Sets PWM trigger mode.  If true, synchronize the control loops with the PWM
  // pulses.  The sensors are sampled 50 uS after the falling edge of the PWM
  // pulse.
  void set_pwm_trigger(bool trigger) { pwm_trigger_ = trigger; }

  // Stops the pwm trigger on the next iteration.
  void Quit() { run_ = false; }

  virtual void RunIteration() = 0;
  // Runs the DMA iteration after the synchronizer.  This only gets run if a
  // sensor has been added to DMA.
  virtual void RunDmaIteration() {}

 protected:
  // Copies a DMAEncoder to a IndexPosition with the correct unit and direction
  // changes.
  void CopyPosition(const ::frc971::wpilib::DMAEncoder &encoder,
                    ::frc971::IndexPositionT *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->encoder =
        multiplier * encoder_translate(encoder.polled_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
    position->latched_encoder =
        multiplier * encoder_translate(encoder.last_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
    position->index_pulses = encoder.index_posedge_count();
  }

  // Copies a AbsoluteEncoderAndPotentiometer to a PotAndAbsolutePosition with
  // the correct unit and direction changes.
  void CopyPosition(
      const ::frc971::wpilib::AbsoluteEncoderAndPotentiometer &encoder,
      ::frc971::PotAndAbsolutePositionT *position,
      double encoder_counts_per_revolution, double encoder_ratio,
      ::std::function<double(double)> potentiometer_translate, bool reverse,
      double pot_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->pot = multiplier * potentiometer_translate(
                                     encoder.ReadPotentiometerVoltage()) +
                    pot_offset;
    position->encoder =
        multiplier * encoder_translate(encoder.ReadRelativeEncoder(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);

    position->absolute_encoder =
        (reverse ? (1.0 - encoder.ReadAbsoluteEncoder())
                 : encoder.ReadAbsoluteEncoder()) *
        encoder_ratio * (2.0 * M_PI);
  }

  // Copies a DMAEdgeCounter to a HallEffectAndPosition with the correct unit
  // and direction changes.
  void CopyPosition(const ::frc971::wpilib::DMAEdgeCounter &counter,
                    ::frc971::HallEffectAndPositionT *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->encoder =
        multiplier * encoder_translate(counter.polled_encoder(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
    position->current = !counter.polled_value();
    position->posedge_count = counter.negative_count();
    position->negedge_count = counter.positive_count();
    position->posedge_value =
        multiplier * encoder_translate(counter.last_negative_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
    position->negedge_value =
        multiplier * encoder_translate(counter.last_positive_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);
  }

  // Copies a Absolute Encoder with the correct unit
  // and direction changes.
  void CopyPosition(
      const ::frc971::wpilib::AbsoluteEncoder &encoder,
      ::frc971::AbsolutePositionT *position,
      double encoder_counts_per_revolution, double encoder_ratio,
      bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->encoder =
        multiplier * encoder_translate(encoder.ReadRelativeEncoder(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio);

    position->absolute_encoder =
        (reverse ? (1.0 - encoder.ReadAbsoluteEncoder())
                 : encoder.ReadAbsoluteEncoder()) *
        encoder_ratio * (2.0 * M_PI);
  }

  void CopyPosition(
      const ::frc971::wpilib::DMAEncoderAndPotentiometer &encoder,
      ::frc971::PotAndIndexPositionT *position,
      ::std::function<double(int32_t)> encoder_translate,
      ::std::function<double(double)> potentiometer_translate, bool reverse,
      double pot_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->encoder =
        multiplier * encoder_translate(encoder.polled_encoder_value());
    position->pot = multiplier * potentiometer_translate(
                                     encoder.polled_potentiometer_voltage()) +
                    pot_offset;
    position->latched_encoder =
        multiplier * encoder_translate(encoder.last_encoder_value());
    position->latched_pot =
        multiplier *
            potentiometer_translate(encoder.last_potentiometer_voltage()) +
        pot_offset;
    position->index_pulses = encoder.index_posedge_count();
  }

  double encoder_translate(int32_t value, double counts_per_revolution,
                           double ratio) {
    return static_cast<double>(value) / counts_per_revolution * ratio *
           (2.0 * M_PI);
  }

  ::aos::EventLoop *event_loop_;
  ::aos::Sender<::aos::RobotState> robot_state_sender_;

  frc::DigitalGlitchFilter fast_encoder_filter_, medium_encoder_filter_;

  ::std::unique_ptr<frc::Encoder> drivetrain_left_encoder_,
      drivetrain_right_encoder_;

 private:
  // Gets called right before the DMA synchronizer is up and running.
  virtual void Start() {}

  // Sets up everything during startup.
  void DoStart();

  // Runs a single iteration.
  void Loop(int iterations);

  // Returns the monotonic time of the start of the first PWM cycle.
  // Returns min_time if no start time could be calculated.
  monotonic_clock::time_point GetPWMStartTime();

  bool pwm_trigger_ = false;

  ::std::unique_ptr<::frc971::wpilib::DMASynchronizer> dma_synchronizer_;

  ::std::atomic<bool> run_{true};
  ::frc::DriverStation *ds_;

  const int32_t my_pid_;

  // Pointer to the phased loop handler used to modify the wakeup.
  ::aos::PhasedLoopHandler *phased_loop_handler_;

  // Last time we got called.
  ::aos::monotonic_clock::time_point last_monotonic_now_ =
      ::aos::monotonic_clock::min_time;
  // The current period.
  chrono::microseconds period_ = chrono::microseconds(5000);
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_SENSOR_READER_H_
