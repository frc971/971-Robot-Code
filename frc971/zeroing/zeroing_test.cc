#include <unistd.h>

#include <memory>

#include <random>

#include "gtest/gtest.h"
#include "frc971/zeroing/zeroing_queue.q.h"
#include "frc971/zeroing/zeroing.h"
#include "aos/common/queue_testutils.h"
#include "aos/common/util/thread.h"
#include "aos/common/die.h"

namespace frc971 {
namespace zeroing {

const int kSeed1 = 0;
const int kSeed2 = 3;

class NoiseGenerator {
 public:
  virtual double AddNoiseToSample(double sample) = 0;
};

class NoNoise : public NoiseGenerator {
 public:
  double AddNoiseToSample(double sample) { return sample; }
};

class FloorNoise : public NoiseGenerator {
 public:
  FloorNoise(double accuracy) : accuracy_(accuracy) {}

  double AddNoiseToSample(double sample) {
    return accuracy_ * ((int)(sample / accuracy_));
  }

 private:
  double accuracy_;
};

class GaussianNoise : public NoiseGenerator {
 public:
  GaussianNoise(unsigned int seed, double stddev)
      : generator_(seed), distribution_(0.0, stddev) {}

  double AddNoiseToSample(double sample) {
    return sample + distribution_(generator_);
  }

 private:
  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
};

class ZeroingEstimatorSimulator {
 public:
  ZeroingEstimatorSimulator(double start_pos, double index_diff,
                            NoiseGenerator& noise, int filter_size = 30)
      : estimator_(index_diff, filter_size), noise_generator_(noise) {
    cur_index_segment_ = (int)(start_pos / index_diff);
    index_diff_ = index_diff;
    start_pos_ = start_pos;
    cur_pos_ = start_pos;
    index_count_ = 0;
    encoder_slip_ = 0;

    // Initialize the ZeroingEstimator instance with the first sensor readings.
    estimator_.UpdateEstimate(getInfo());
  }

  void MoveTo(double new_pos) {
    int new_index = (int)(new_pos - encoder_slip_) / index_diff_;
    if (new_index < cur_index_segment_) {
      cur_index_ = new_index + 1;
      index_count_++;
    }
    if (new_index > cur_index_segment_) {
      cur_index_ = new_index;
      index_count_++;
    }
    cur_index_segment_ = new_index;
    cur_pos_ = new_pos;

    estimator_.UpdateEstimate(getInfo());
  }

  // Simulate the encoder slipping by `slip'.
  void MoveWithEncoderSlip(double slip) {
    encoder_slip_ += slip;

    MoveTo(cur_pos_ + slip);

    estimator_.UpdateEstimate(getInfo());
  }

  ZeroingInfo getInfo() {
    ZeroingInfo estimate;
    estimate.pot = noise_generator_.AddNoiseToSample(cur_pos_);
    if (index_count_ == 0) {
      estimate.index_encoder = 0.0;
    } else {
      estimate.index_encoder = cur_index_ * index_diff_ - start_pos_;
    }
    estimate.index_count = index_count_;
    estimate.encoder = cur_pos_ - start_pos_ - encoder_slip_;
    return estimate;
  }

  double getEstimate(void) { return estimator_.getPosition(); }

 private:
  int index_count_;
  int cur_index_;
  int cur_index_segment_;
  double index_diff_;
  double start_pos_;
  double cur_pos_;
  double encoder_slip_;
  ZeroingEstimator estimator_;
  NoiseGenerator& noise_generator_;
};

class QueueTest : public ::testing::Test {
 protected:
  void SetUp() override { aos::SetDieTestMode(true); }

  aos::common::testing::GlobalCoreInstance my_core;
  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ::aos::Queue<TestMessage> my_test_queue;

  QueueTest() : my_test_queue(".frc971.zeroing.test_queue") {}
};

TEST_F(QueueTest, FetchBlocking) {
  // Make sure that the queue works.
  my_test_queue.MakeWithBuilder().test_int(0x971).Send();
  EXPECT_TRUE(my_test_queue.FetchNext());
}

TEST_F(QueueTest, SimpleStep) {
  FloorNoise floored_pot(0.25);
  ZeroingEstimatorSimulator sim(3.6, 1.0, floored_pot, 1);

  // The first estimate should be 3.5 since that's the only reliable number we
  // have. (i.e. 3.6 rounded down to the nearest 0.25 multiple.
  ASSERT_NEAR(3.5, sim.getEstimate(), 0.001);

  // Next we'll move to 3.65 which should still give us a reading of 3.50. This
  // is because we're just using one sample to "filter" the noise. In this case
  // the filter would take 3.5 from the pot value and subract the encoder
  // reading of 0.05. In order to come up with an accurate estimate, we add the
  // encoder value back in.
  sim.MoveTo(3.65);
  ASSERT_NEAR(3.50, sim.getEstimate(), 0.001);

  // Now we move to 3.80 which should give us the a reading of 3.70. Similar to
  // the above scenario we've now moved 0.20 in total which is the reading of
  // the encoder. Unfortunately, we can't use the encoder value yet since we
  // don't know where it is relative to the index pulse.
  sim.MoveTo(3.80);
  ASSERT_NEAR(3.75, sim.getEstimate(), 0.001);

  // We move past the 4.00 mark right to 4.10. The pot value will read 4.00,
  // the encoder reads 0.5 and the index pulse sample will read 0.4. Now we
  // know that we are 0.1 past the 4.00 mark.
  sim.MoveTo(4.10);
  ASSERT_NEAR(4.10, sim.getEstimate(), 0.001);

  // We move back to 3.80 and now we should have an accurate reading. The pot
  // value reads 3.75, the encoder reads 0.2 and the index pulse is again set
  // at 0.4. Thus we can deduce that we're 0.2 below the 4.00 mark (i.e. at
  // 3.80)
  sim.MoveTo(3.80);
  ASSERT_NEAR(3.80, sim.getEstimate(), 0.001);

  // Just for kicks we'll move back to a value of 2.56 which the estimator
  // should be able to calculate.
  sim.MoveTo(2.56);
  ASSERT_NEAR(2.56, sim.getEstimate(), 0.001);
}

TEST_F(QueueTest, TestMovingAverageFilter) {
  GaussianNoise pot_noise(kSeed1, 0.5 / 3.0);
  ZeroingEstimatorSimulator sim(3.6, 1.0, pot_noise);

  // The zeroing code is supposed to perform some filtering on the difference
  // between the potentiometer value and the encoder value. We assume that 300
  // samples are sufficient to have updated the filter.
  for (int i = 0; i < 300; i++) {
    sim.MoveTo(3.3);
  }
  ASSERT_NEAR(3.3, sim.getEstimate(), 0.1);

  for (int i = 0; i < 300; i++) {
    sim.MoveTo(3.9);
  }
  ASSERT_NEAR(3.9, sim.getEstimate(), 0.1);
}

TEST_F(QueueTest, TestLotsOfMovement) {
  double index_diff = 1.00;
  GaussianNoise pot_noise(kSeed2, index_diff / 3.0);
  ZeroingEstimatorSimulator sim(3.6, index_diff, pot_noise);

  // The zeroing code is supposed to perform some filtering on the difference
  // between the potentiometer value and the encoder value. We assume that 300
  // samples are sufficient to have updated the filter.
  for (int i = 0; i < 300; i++) {
    sim.MoveTo(3.6);
  }
  ASSERT_NEAR(3.6, sim.getEstimate(), 0.1);

  // With a single index pulse the zeroing estimator should be able to lock
  // onto the true value of the position.
  sim.MoveTo(4.01);
  ASSERT_NEAR(4.01, sim.getEstimate(), 0.001);

  sim.MoveTo(4.99);
  ASSERT_NEAR(4.99, sim.getEstimate(), 0.001);

  sim.MoveTo(3.99);
  ASSERT_NEAR(3.99, sim.getEstimate(), 0.001);

  sim.MoveTo(3.01);
  ASSERT_NEAR(3.01, sim.getEstimate(), 0.001);

  sim.MoveTo(13.55);
  ASSERT_NEAR(13.55, sim.getEstimate(), 0.001);
}

TEST_F(QueueTest, TestDifferentIndexDiffs) {
  double index_diff = 0.89;
  GaussianNoise pot_noise(kSeed2, index_diff / 3.0);
  ZeroingEstimatorSimulator sim(3.5 * index_diff, index_diff, pot_noise);

  // The zeroing code is supposed to perform some filtering on the difference
  // between the potentiometer value and the encoder value. We assume that 300
  // samples are sufficient to have updated the filter.
  for (int i = 0; i < 300; i++) {
    sim.MoveTo(3.5 * index_diff);
  }
  ASSERT_NEAR(3.5 * index_diff, sim.getEstimate(), 0.1);

  // With a single index pulse the zeroing estimator should be able to lock
  // onto the true value of the position.
  sim.MoveTo(4.01);
  ASSERT_NEAR(4.01, sim.getEstimate(), 0.001);

  sim.MoveTo(4.99);
  ASSERT_NEAR(4.99, sim.getEstimate(), 0.001);

  sim.MoveTo(3.99);
  ASSERT_NEAR(3.99, sim.getEstimate(), 0.001);

  sim.MoveTo(3.01);
  ASSERT_NEAR(3.01, sim.getEstimate(), 0.001);

  sim.MoveTo(13.55);
  ASSERT_NEAR(13.55, sim.getEstimate(), 0.001);
}

}  // namespace zeroing
}  // namespace frc971
