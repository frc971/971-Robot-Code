#ifndef FRC971_INPUT_SENSOR_PACKER_H_
#define FRC971_INPUT_SENSOR_PACKER_H_

#include "aos/common/libstdc++/memory"

#include "WPILib/Task.h"
#include "WPILib/Encoder.h"
#include "WPILib/DigitalInput.h"
#include "WPILib/Counter.h"
#include "WPILib/CounterBase.h"
#include "WPILib/AnalogChannel.h"
#include "WPILib/AnalogTrigger.h"

#include "aos/common/mutex.h"
#include "aos/crio/shared_libs/interrupt_notifier.h"
#include "aos/common/sensors/sensor_packer.h"
#include "aos/crio/hardware/counter.h"
#include "aos/crio/hardware/digital_source.h"

#include "frc971/queues/sensor_values.h"

namespace frc971 {

class SensorPacker
    : public ::aos::sensors::SensorPackerInterface<sensor_values> {
 public:
  SensorPacker();

  virtual void PackInto(sensor_values *values);

 private:
  // Used for passing ReadEncoder all of the information that it needs.
  struct EncoderReadData {
    EncoderReadData(::aos::crio::hardware::Counter *counter,
                    int32_t *output,
                    ::aos::Mutex *sync)
        : counter(counter), output(output), sync(sync) {}

    ::aos::crio::hardware::Counter *const counter;
    int32_t *const output;
    ::aos::Mutex *const sync;
  };

  // Handler function for an ::aos::crio::InterruptNotifier that reads an
  // encoder and (synchronized on a ::aos::Mutex) writes the value into a
  // pointer.
  static void ReadEncoder(EncoderReadData *data);

  // A whole day.
  // Used for passing to WPILib's WaitForInterrupts.
  static const float kInterruptTimeout = 60 * 60 * 24;

  static void StaticTopDiscEdgeReader(void *self) {
    static_cast<SensorPacker *>(self)->TopDiscEdgeReader();
  }
  static void StaticBottomDiscEdgeReader(void *self) {
    static_cast<SensorPacker *>(self)->BottomDiscEdgeReader();
  }

  void TopDiscEdgeReader();
  void BottomDiscEdgeReader();

  ::std::unique_ptr< ::aos::crio::hardware::Counter> drive_left_counter_;
  ::std::unique_ptr< ::aos::crio::hardware::Counter> drive_right_counter_;

  ::std::unique_ptr< ::aos::crio::hardware::Counter> wrist_counter_;
  ::std::unique_ptr< ::aos::crio::hardware::DigitalSource> wrist_hall_effect_;
  int32_t wrist_edge_position_;
  ::aos::Mutex wrist_sync_;
  ::aos::crio::InterruptNotifier<EncoderReadData> wrist_notifier_;

  ::std::unique_ptr< ::aos::crio::hardware::Counter> angle_adjust_counter_;
  ::std::unique_ptr< ::aos::crio::hardware::DigitalSource> angle_adjust_middle_hall_effect_;
  ::std::unique_ptr< ::aos::crio::hardware::DigitalSource> angle_adjust_bottom_hall_effect_;
  int32_t angle_adjust_middle_edge_position_;
  int32_t angle_adjust_bottom_edge_position_;
  ::aos::Mutex angle_adjust_sync_;
  ::aos::crio::InterruptNotifier<EncoderReadData>
      angle_adjust_middle_notifier_;
  ::aos::crio::InterruptNotifier<EncoderReadData>
      angle_adjust_bottom_notifier_;

  ::std::unique_ptr< ::aos::crio::hardware::Counter> shooter_counter_;

  ::std::unique_ptr< ::aos::crio::hardware::CounterCounter> index_counter_;
  Task top_disc_edge_task_;
  ::aos::Mutex top_disc_edge_sync_;
  ::std::unique_ptr< ::AnalogTrigger> top_disc_;
  ::std::unique_ptr< ::aos::crio::hardware::DigitalSource> top_disc_posedge_output_;
  ::std::unique_ptr< ::aos::crio::hardware::DigitalSource> top_disc_negedge_output_;
  int32_t top_disc_posedge_count_;
  int32_t top_disc_negedge_count_;
  int32_t top_disc_posedge_position_;
  int32_t top_disc_negedge_position_;
  Task bottom_disc_edge_task_;
  ::aos::Mutex bottom_disc_edge_sync_;
  ::std::unique_ptr< ::aos::crio::hardware::DigitalSource> bottom_disc_;
  int32_t bottom_disc_posedge_count_;
  int32_t bottom_disc_negedge_count_;
  int32_t bottom_disc_negedge_wait_count_;
  int32_t bottom_disc_negedge_wait_position_;
};

}  // namespace frc971

#endif  // FRC971_INPUT_SENSOR_PACKER_H_
