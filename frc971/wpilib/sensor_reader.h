#ifndef FRC971_WPILIB_SENSOR_READER_H_
#define FRC971_WPILIB_SENSOR_READER_H_

#include <atomic>
#include <chrono>

#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DigitalInput.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace frc971 {
namespace wpilib {

class SensorReader {
 public:
  SensorReader();
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

  // Sets the pwm trigger.
  void set_pwm_trigger(::std::unique_ptr<frc::DigitalInput> pwm_trigger);

  // Stops the pwm trigger on the next iteration.
  void Quit() { run_ = false; }

  virtual void RunIteration() = 0;
  // Runs the DMA iteration after the synchronizer.  This only gets run if a
  // sensor has been added to DMA.
  virtual void RunDmaIteration() {}

  void operator()();

 protected:
  // Copies a DMAEncoder to a IndexPosition with the correct unit and direction
  // changes.
  void CopyPosition(const ::frc971::wpilib::DMAEncoder &encoder,
                    ::frc971::IndexPosition *position,
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
      ::frc971::PotAndAbsolutePosition *position,
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
                    ::frc971::HallEffectAndPosition *position,
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
      ::frc971::AbsolutePosition *position,
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
      ::frc971::PotAndIndexPosition *position,
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

  frc::DigitalGlitchFilter fast_encoder_filter_, medium_encoder_filter_;

  ::std::unique_ptr<frc::Encoder> drivetrain_left_encoder_,
      drivetrain_right_encoder_;

 private:
  // Gets called right before the DMA synchronizer is up and running.
  virtual void Start() {}

  // Uses the pwm trigger to find the pwm cycle width and offset for that
  // iteration.
  void RunPWMDetecter();

  ::std::unique_ptr<frc::DigitalInput> pwm_trigger_;

  // Mutex to manage access to the period and tick time variables.
  ::aos::stl_mutex tick_time_mutex_;
  monotonic_clock::time_point last_tick_time_monotonic_timepoint_ =
      monotonic_clock::min_time;
  chrono::nanoseconds last_period_;

  ::std::unique_ptr<::frc971::wpilib::DMASynchronizer> dma_synchronizer_;

  ::std::atomic<bool> run_{true};
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_SENSOR_READER_H_
