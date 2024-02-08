#include "frc971/control_loops/encoder_fault_detector.h"

#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer.h"

namespace frc971 {
namespace control_loops {
namespace testing {

// Test for simulating if encoder values are idle.
TEST(EncoderFaultDetector, Idle) {
  EncoderFaultDetector<2> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 2> falcon_positions = {0.0, 0.0};

  for (int i = 0; i < 10; i++) {
    detector.Iterate(encoder_position, falcon_positions, t);
    t += std::chrono::milliseconds(5);
  }

  EXPECT_FALSE(detector.isfaulted());
}

// Test for simulating if we have three motors
TEST(EncoderFaultDetector, ThreeMotors) {
  EncoderFaultDetector<3> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 3> falcon_positions = {0.0, 0.0, 0.0};

  for (int i = 0; i < 10; i++) {
    detector.Iterate(encoder_position, falcon_positions, t);
    t += std::chrono::milliseconds(5);
  }

  EXPECT_FALSE(detector.isfaulted());
}

// Test for simulating faulting with three motors
TEST(EncoderFaultDetector, FaultThreeMotors) {
  EncoderFaultDetector<3> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 3> falcon_positions = {0.0, 0.0, 0.0};

  for (int i = 0; i < 10; i++) {
    for (double &falcon : falcon_positions) {
      falcon++;
    }
    detector.Iterate(encoder_position, falcon_positions, t);
    t += std::chrono::milliseconds(5);
  }

  EXPECT_TRUE(detector.isfaulted());
}

// Test for simulating if encoder values are increasing.
TEST(EncoderFaultDetector, Increasing) {
  EncoderFaultDetector<2> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 2> falcon_positions = {0.0, 0.0};

  for (int i = 0; i < 10; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    encoder_position++;
    for (double &falcon : falcon_positions) {
      falcon++;
    }
    t += std::chrono::milliseconds(5);
  }

  EXPECT_FALSE(detector.isfaulted());
}

// Test for simulating if encoder values are decreasing.
TEST(EncoderFaultDetector, Decreasing) {
  EncoderFaultDetector<2> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 2> falcon_positions = {0.0, 0.0};

  for (int i = 0; i < 10; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    encoder_position--;
    for (double &falcon : falcon_positions) {
      falcon--;
    }
    t += std::chrono::milliseconds(5);
  }

  EXPECT_FALSE(detector.isfaulted());
}

// Test for simulating if only falcon values are increasing.
TEST(EncoderFaultDetector, FalconsOnlyIncrease) {
  EncoderFaultDetector<2> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 2> falcon_positions = {0.0, 0.0};

  for (int i = 0; i < 5; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    for (double &falcon : falcon_positions) {
      falcon++;
    }
    t += std::chrono::milliseconds(5);
  }

  EXPECT_TRUE(detector.isfaulted());
}

// Test for simulating if only encoder value is increasing.
TEST(EncoderFaultDetector, EncoderOnlyIncrease) {
  EncoderFaultDetector<2> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 2> falcon_positions = {0.0, 0.0};

  for (int i = 0; i < 5; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    encoder_position++;
    t += std::chrono::milliseconds(5);
  }

  EXPECT_TRUE(detector.isfaulted());
}

// Test for simulating if only one falcon value is increasing at a time.
TEST(EncoderFaultDetector, OnlyOneFalconIncreases) {
  EncoderFaultDetector<2> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 2> falcon_positions = {0.0, 0.0};

  for (int i = 0; i < 5; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    falcon_positions[0] += 0.1;
    t += std::chrono::milliseconds(5);
  }

  for (int i = 0; i < 5; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    falcon_positions[1] += 0.1;
    t += std::chrono::milliseconds(5);
  }

  EXPECT_TRUE(detector.isfaulted());
}

// Test checks that the detector stays faulted after a fault if the encoder
// positions align again
TEST(EncoderFaultDetector, StaysFaulted) {
  EncoderFaultDetector<2> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 2> falcon_positions = {0.0, 0.0};

  for (int i = 0; i < 5; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    encoder_position += 0.1;
    t += std::chrono::milliseconds(5);
  }

  EXPECT_TRUE(detector.isfaulted());

  for (int i = 0; i < 5; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    encoder_position += 0.1;
    for (double &falcon : falcon_positions) {
      falcon += 0.1;
    }
    t += std::chrono::milliseconds(5);
  }

  EXPECT_TRUE(detector.isfaulted());
}

// Tests that after 10 milliseconds updates will not register
TEST(EncoderFaultDetector, NoUpdateForTooLong) {
  EncoderFaultDetector<2> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 2> falcon_positions = {0.0, 0.0};

  for (int i = 0; i < 5; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    encoder_position += 0.1;
    t += std::chrono::milliseconds(11);
  }

  EXPECT_FALSE(detector.isfaulted());
}

// Tests if populate status function is working as expected
TEST(EncoderFaultDetector, PopulateStatus) {
  EncoderFaultDetector<2> detector;
  aos::monotonic_clock::time_point t;

  double encoder_position = 0.0;
  aos::SizedArray<double, 2> falcon_positions = {0.0, 0.0};

  for (int i = 0; i < 10; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    encoder_position++;
    for (double &falcon : falcon_positions) {
      falcon++;
    }
    t += std::chrono::milliseconds(5);
  }

  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(detector.PopulateStatus(&fbb));
  aos::FlatbufferDetachedBuffer<EncoderFaultStatus> result = fbb.Release();

  EXPECT_EQ("{ \"faulted\": false }", aos::FlatbufferToJson(result));

  for (int i = 0; i < 5; ++i) {
    detector.Iterate(encoder_position, falcon_positions, t);
    encoder_position++;
    t += std::chrono::milliseconds(5);
  }

  fbb.Finish(detector.PopulateStatus(&fbb));
  result = fbb.Release();

  EXPECT_EQ("{ \"faulted\": true }", aos::FlatbufferToJson(result));
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971