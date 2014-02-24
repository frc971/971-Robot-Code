#include <inttypes.h>

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/wrapping_counter.h"
#include "aos/common/time.h"
#include "aos/common/logging/queue_logging.h"

#include "bbb/sensor_reader.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/gyro_angle.q.h"
#include "frc971/constants.h"
#include "frc971/queues/to_log.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/control_loops/claw/claw.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain;
using ::frc971::sensors::gyro;
using ::aos::util::WrappingCounter;

namespace frc971 {
namespace {

struct State {
  struct HallEffectCounters {
    WrappingCounter posedges, negedges;
  };

  HallEffectCounters plunger, pusher_distal, pusher_proximal, latch;

  struct SingleClawState {
    HallEffectCounters front, calibration, back;
  } top_claw, bottom_claw;
};

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in)
      / (256.0 /*cpr*/ * 4.0 /*quad*/)
      * (18.0 / 50.0 /*output stage*/) * (56.0 / 30.0 /*encoder gears*/)
      // * constants::GetValues().drivetrain_encoder_ratio
      * (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

// Translates values from the ADC into voltage.
// TODO(brian): Fix this.
double adc_translate(uint16_t in) {
  static const double kVcc = 5;
  //static const double kR1 = 5, kR2 = 6.65;
  static const uint16_t kMaximumValue = 0x3FF;
  return (kVcc * static_cast<double>(in) / static_cast<double>(kMaximumValue));
}

double gyro_translate(int64_t in) {
  return in / 16.0 / 1000.0 / (180.0 / M_PI);
}

double battery_translate(uint16_t in) {
  static const double kDividerBig = 98.9, kDividerSmall = 17.8;
  return adc_translate(in) * kDividerBig / kDividerSmall;
}

double hall_translate(const constants::ShifterHallEffect &k, uint16_t in) {
  const double voltage = adc_translate(in);
  return (voltage - k.low) / (k.high - k.low);
}

double claw_translate(int32_t in) {
  return static_cast<double>(in)
      / (256.0 /*cpr*/ * 4.0 /*quad*/)
      / (18.0 / 48.0 /*encoder gears*/)
      / (12.0 / 60.0 /*chain reduction*/)
      * (M_PI / 180.0)
      * 2.0 /*TODO(austin): Debug this, encoders read too little*/;
}

double shooter_translate(int32_t in) {
  return static_cast<double>(in)
      / (256.0 /*cpr*/ * 4.0 /*quad*/)
      * 16 /*sprocket teeth*/ * 0.375 /*chain pitch*/
      * (2.54 / 100.0 /*in to m*/);
}

template<typename Structure>
void CopyHallEffectEdges(Structure *output,
                         const ::bbb::HallEffectEdges &input,
                         State::HallEffectCounters *state) {
  output->posedge_count = state->posedges.Update(input.posedges);
  output->negedge_count = state->negedges.Update(input.negedges);
}

void CopyClawPosition(control_loops::HalfClawPosition *output,
                      const ::bbb::SingleClawPosition &input,
                      State::SingleClawState *state,
                      bool reversed) {
  CopyHallEffectEdges(&output->front, input.front, &state->front);
  output->front.current = input.bools.front;
  CopyHallEffectEdges(&output->calibration, input.calibration,
                      &state->calibration);
  output->calibration.current = input.bools.calibration;
  CopyHallEffectEdges(&output->back, input.back, &state->back);
  output->back.current = input.bools.back;

  const double multiplier = reversed ? -1.0 : 1.0;

  output->position = multiplier * claw_translate(input.position);
  output->posedge_value = multiplier * claw_translate(input.posedge_position);
  output->negedge_value = multiplier * claw_translate(input.negedge_position);
}

void PacketReceived(const ::bbb::DataStruct *data,
                    const ::aos::time::Time &cape_timestamp,
                    State *state) {
  ::frc971::logging_structs::CapeReading reading_to_log(
      cape_timestamp.sec(), cape_timestamp.nsec(),
      data->main.ultrasonic_pulse_length);
  LOG_STRUCT(DEBUG, "cape reading", reading_to_log);
  bool bad_gyro;
  if (data->uninitialized_gyro) {
    LOG(DEBUG, "uninitialized gyro\n");
    bad_gyro = true;
  } else if (data->zeroing_gyro) {
    LOG(DEBUG, "zeroing gyro\n");
    bad_gyro = true;
  } else if (data->bad_gyro) {
    LOG(ERROR, "bad gyro\n");
    bad_gyro = true;
    gyro.MakeWithBuilder().angle(0).Send();
  } else if (data->old_gyro_reading) {
    LOG(WARNING, "old/bad gyro reading\n");
    bad_gyro = true;
  } else {
    bad_gyro = false;
  }

  if (!bad_gyro) {
    gyro.MakeWithBuilder()
        .angle(gyro_translate(data->gyro_angle))
        .Send();
  }

  drivetrain.position.MakeWithBuilder()
      .right_encoder(drivetrain_translate(data->main.right_drive))
      .left_encoder(-drivetrain_translate(data->main.left_drive))
      .left_shifter_position(hall_translate(constants::GetValues().left_drive,
                                            data->main.left_drive_hall))
      .right_shifter_position(hall_translate(
              constants::GetValues().right_drive, data->main.right_drive_hall))
      .battery_voltage(battery_translate(data->main.battery_voltage))
      .Send();

  {
    auto claw_position = control_loops::claw_queue_group.position.MakeMessage();
    CopyClawPosition(&claw_position->top, data->main.top_claw,
                     &state->top_claw, false);
    CopyClawPosition(&claw_position->bottom, data->main.bottom_claw,
                     &state->bottom_claw, true);
    claw_position.Send();
  }

  {
    auto shooter_position =
        control_loops::shooter_queue_group.position.MakeMessage();

    shooter_position->plunger = data->main.bools.plunger;
    CopyHallEffectEdges(&shooter_position->pusher_distal,
                        data->main.pusher_distal, &state->pusher_distal);
    shooter_position->pusher_distal.current = data->main.bools.pusher_distal;
    CopyHallEffectEdges(&shooter_position->pusher_proximal,
                        data->main.pusher_proximal, &state->pusher_proximal);
    shooter_position->pusher_proximal.current =
        data->main.bools.pusher_proximal;
    shooter_position->latch = data->main.bools.latch;

    shooter_position->position = shooter_translate(data->main.shooter_position);
    shooter_position->pusher_distal.posedge_value =
        shooter_translate(data->main.pusher_distal_posedge_position);
    shooter_position->pusher_proximal.posedge_value =
        shooter_translate(data->main.pusher_proximal_posedge_position);

    shooter_position.Send();
  }
}

}  // namespace
}  // namespace frc971

int main() {
  ::aos::Init(::bbb::SensorReader::kRelativePriority);
  ::bbb::SensorReader reader("comp");
  ::frc971::State state;
  while (true) {
    ::frc971::PacketReceived(reader.ReadPacket(), reader.GetCapeTimestamp(),
                             &state);
  }
  ::aos::Cleanup();
}
