#include "gtest/gtest.h"

#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/zeroing/zeroing.h"

namespace frc971 {
namespace zeroing {
namespace testing {

using control_loops::PositionSensorSimulator;
using FBB = flatbuffers::FlatBufferBuilder;

constexpr size_t kSampleSize = 30;
constexpr double kAcceptableUnzeroedError = 0.2;
constexpr double kIndexErrorFraction = 0.3;
constexpr size_t kMovingBufferSize = 3;

class ZeroingTest : public ::testing::Test {
 protected:
  template <typename T>
  double GetEstimatorPosition(T *estimator) {
    FBB fbb;
    fbb.Finish(estimator->GetEstimatorState(&fbb));
    return flatbuffers::GetRoot<typename T::State>(fbb.GetBufferPointer())
        ->position();
  }
};

}  // namespace testing
}  // namespace zeroing
}  // namespace frc971
