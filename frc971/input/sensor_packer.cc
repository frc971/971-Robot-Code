#include "frc971/input/sensor_packer.h"

#include <arpa/inet.h>

#include "aos/common/inttypes.h"
#include "aos/common/time.h"

#include "frc971/control_loops/index/index.h"

using ::aos::MutexLocker;

namespace frc971 {

SensorPacker::SensorPacker() : lencoder(1, 2), rencoder(3, 4) {
  lencoder.Start();
  rencoder.Start();

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
  values->lencoder = htonl(-lencoder.GetRaw());
  values->rencoder = -htonl(-rencoder.GetRaw());
}

}  // namespace frc971
