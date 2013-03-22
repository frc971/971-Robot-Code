#include "frc971/input/sensor_packer.h"

#include <arpa/inet.h>

#include "aos/common/inttypes.h"
#include "aos/common/time.h"

#include "frc971/control_loops/index/index.h"

using ::std::unique_ptr;
namespace hardware = ::aos::crio::hardware;

namespace frc971 {

SensorPacker::SensorPacker()
    : drive_left_counter_(::hardware::Counter::Create(
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(3)),
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(4)),
              ::CounterBase::EncodingType::k4X)),
      drive_right_counter_(::hardware::Counter::Create(
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(1)),
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(2)),
              ::CounterBase::EncodingType::k4X)),

      wrist_counter_(::hardware::Counter::Create(
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(5)),
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(6)),
              ::CounterBase::EncodingType::k4X)),
      wrist_hall_effect_(new ::hardware::AnalogTriggerOutput(
              unique_ptr< ::AnalogTrigger>(
                  new ::AnalogTrigger(1)),
              ::AnalogTriggerOutput::Type::kState)),
      wrist_edge_position_(0),
      wrist_notifier_(ReadEncoder, wrist_hall_effect_->source(),
                      new EncoderReadData(wrist_counter_.get(),
                      &wrist_edge_position_, &wrist_sync_)),

      angle_adjust_counter_(::hardware::Counter::Create(
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(7)),
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(8)),
              ::CounterBase::EncodingType::k2X)),
      angle_adjust_middle_hall_effect_(new ::hardware::AnalogTriggerOutput(
              unique_ptr< ::AnalogTrigger>(
                  new ::AnalogTrigger(5)),
              ::AnalogTriggerOutput::Type::kState)),
      angle_adjust_bottom_hall_effect_(new ::hardware::AnalogTriggerOutput(
              unique_ptr< ::AnalogTrigger>(
                  new ::AnalogTrigger(3)),
              ::AnalogTriggerOutput::Type::kState)),
      angle_adjust_middle_notifier_(
          ReadEncoder,
          angle_adjust_middle_hall_effect_->source(),
          new EncoderReadData(angle_adjust_counter_.get(),
                             &angle_adjust_middle_edge_position_,
                              &angle_adjust_sync_)),
      angle_adjust_bottom_notifier_(
          ReadEncoder,
          angle_adjust_bottom_hall_effect_->source(),
          new EncoderReadData(angle_adjust_counter_.get(),
                              &angle_adjust_bottom_edge_position_,
                              &angle_adjust_sync_)),

      shooter_counter_(::hardware::Counter::Create(
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(9)),
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(10)),
              ::CounterBase::EncodingType::k4X)),

      index_counter_(new ::hardware::CounterCounter(
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(11)),
              unique_ptr< ::hardware::DigitalSource>(
                  new ::hardware::DigitalInput(12)),
              ::CounterBase::EncodingType::k2X)),
      top_disc_edge_task_("TopDsc",
                          reinterpret_cast<FUNCPTR>(
                              StaticTopDiscEdgeReader),
                          96),
      top_disc_(new ::AnalogTrigger(2)),
      top_disc_posedge_output_(new ::hardware::AnalogTriggerOutput(
              unique_ptr< ::AnalogTriggerOutput>(
                  top_disc_->CreateOutput(
                      ::AnalogTriggerOutput::Type::kState)))),
      top_disc_negedge_output_(new ::hardware::AnalogTriggerOutput(
              unique_ptr< ::AnalogTriggerOutput>(
                  top_disc_->CreateOutput(
                      ::AnalogTriggerOutput::Type::kState)))),
      top_disc_posedge_count_(0),
      top_disc_negedge_count_(0),
      top_disc_posedge_position_(0),
      top_disc_negedge_position_(0),
      bottom_disc_edge_task_("BotDsc",
                             reinterpret_cast<FUNCPTR>(
                                 StaticBottomDiscEdgeReader),
                             96),
      bottom_disc_(new ::hardware::DigitalInput(13)),
      bottom_disc_posedge_count_(0),
      bottom_disc_negedge_count_(0),
      bottom_disc_negedge_wait_count_(0),
      bottom_disc_negedge_wait_position_(0) {
  // 7, 8 = angle, down = -
  // 5, 6 = wrist, down = +
  // drive = flipped, l/r = flipped
  // 9, 10 = angle adjust, out = -
  // 11, 12 = indexing, up = -
  //   positive result should be up in the end
  drive_left_counter_->counter_base()->Start();
  drive_right_counter_->counter_base()->Start();

  shooter_counter_->counter_base()->Start();

  wrist_counter_->counter_base()->Start();
  wrist_hall_effect_->source()->SetUpSourceEdge(true, true);
  wrist_notifier_.Start();

  angle_adjust_counter_->counter_base()->Start();
  angle_adjust_middle_hall_effect_->source()->SetUpSourceEdge(true, true);
  angle_adjust_bottom_hall_effect_->source()->SetUpSourceEdge(true, true);
  angle_adjust_middle_notifier_.Start();
  angle_adjust_bottom_notifier_.Start();

  index_counter_->counter()->SetExternalDirectionMode();
  index_counter_->counter_base()->Start();
  top_disc_->SetLimitsVoltage(
      ::hardware::AnalogTriggerOutput::kDefaultLowerVoltage,
      ::hardware::AnalogTriggerOutput::kDefaultUpperVoltage);
  top_disc_posedge_output_->source()->RequestInterrupts();
  top_disc_posedge_output_->source()->SetUpSourceEdge(false, true);
  top_disc_negedge_output_->source()->RequestInterrupts();
  top_disc_negedge_output_->source()->SetUpSourceEdge(true, false);
  top_disc_edge_task_.Start(reinterpret_cast<uintptr_t>(this));
  // TODO(brians) change this to the correct constant
  aos::time::SleepFor(aos::time::Time::InSeconds(0.25));
  bottom_disc_->source()->RequestInterrupts();
  bottom_disc_->source()->SetUpSourceEdge(true, true);
  bottom_disc_edge_task_.Start(reinterpret_cast<uintptr_t>(this));
  // TODO(brians) change this to the correct constant
  aos::time::SleepFor(aos::time::Time::InSeconds(0.25));

  printf("frc971::SensorPacker started\n");
}

void SensorPacker::ReadEncoder(EncoderReadData *data) {
  int32_t value = data->counter->Get();
  {
    ::aos::MutexLocker locker(data->sync);
    *data->output = value;
  }
}

void SensorPacker::TopDiscEdgeReader() {
  while (true) {
    top_disc_posedge_output_->source()->
        WaitForInterrupt(kInterruptTimeout);
    int32_t position = index_counter_->Get();
    {
      aos::MutexLocker locker(&top_disc_edge_sync_);
      top_disc_posedge_position_ = position;
      ++top_disc_posedge_count_;
    }
    top_disc_negedge_output_->source()->
        WaitForInterrupt(kInterruptTimeout);
    position = index_counter_->Get();
    {
      aos::MutexLocker locker(&top_disc_edge_sync_);
      top_disc_negedge_position_ = position;
      ++top_disc_negedge_count_;
    }
  }
}

void SensorPacker::BottomDiscEdgeReader() {
  static const aos::time::Time kBottomDiscNegedgeSleep =
      aos::time::Time::InSeconds(
          control_loops::IndexMotor::kBottomDiscIndexDelay);
  while (true) {
    bottom_disc_->source()->WaitForInterrupt(kInterruptTimeout);
    if (bottom_disc_->Get()) {
      aos::time::Time start = aos::time::Time::Now();
      {
        aos::MutexLocker locker(&bottom_disc_edge_sync_);
        ++bottom_disc_negedge_count_;
      }
      aos::time::SleepUntil(start + kBottomDiscNegedgeSleep);
      int32_t position = index_counter_->Get();
      {
        aos::MutexLocker locker(&bottom_disc_edge_sync_);
        bottom_disc_negedge_wait_position_ = position;
        ++bottom_disc_negedge_wait_count_;
      }
    } else {
      {
        aos::MutexLocker locker(&bottom_disc_edge_sync_);
        ++bottom_disc_posedge_count_;
      }
    }
  }
}

void SensorPacker::PackInto(sensor_values *values) {
  values->shooter_encoder = shooter_counter_->Get();

  values->drive_left_encoder = htonl(-drive_left_counter_->Get());
  values->drive_right_encoder = -htonl(-drive_right_counter_->Get());

  {
    aos::MutexLocker locker(&wrist_sync_);
    values->wrist_position = wrist_counter_->Get();
    values->wrist_edge_position = wrist_edge_position_;
    values->wrist_hall_effect = wrist_hall_effect_->Get();
  }

  {
    aos::MutexLocker locker(&angle_adjust_sync_);
    values->angle_adjust_position = angle_adjust_counter_->Get();
    values->angle_adjust_middle_edge_position =
        angle_adjust_middle_edge_position_;
    values->angle_adjust_bottom_edge_position =
        angle_adjust_bottom_edge_position_;
    values->angle_adjust_middle_hall_effect =
        angle_adjust_middle_hall_effect_->Get();
    values->angle_adjust_bottom_hall_effect =
        angle_adjust_bottom_hall_effect_->Get();
  }

  values->index_encoder = index_counter_->Get();
  {
    aos::MutexLocker locker(&top_disc_edge_sync_);
    values->top_disc_posedge_count = top_disc_posedge_count_;
    values->top_disc_negedge_count = top_disc_negedge_count_;
    values->top_disc_posedge_position = top_disc_posedge_position_;
    values->top_disc_negedge_position = top_disc_negedge_position_;
    values->top_disc = top_disc_->GetTriggerState();
  }
  {
    aos::MutexLocker locker(&bottom_disc_edge_sync_);
    values->bottom_disc_negedge_wait_position =
        bottom_disc_negedge_wait_position_;
    values->bottom_disc_negedge_count = bottom_disc_negedge_count_;
    values->bottom_disc_negedge_wait_count = bottom_disc_negedge_wait_count_;
    values->bottom_disc_posedge_count = bottom_disc_posedge_count_;
    values->bottom_disc = bottom_disc_->Get();
  }
}

}  // namespace frc971
