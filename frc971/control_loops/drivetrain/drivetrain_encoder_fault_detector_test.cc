#include "frc971/control_loops/drivetrain/drivetrain_encoder_fault_detector.h"

#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"
#include "aos/json_to_flatbuffer.h"

namespace frc971::control_loops::drivetrain::testing {

class EncoderFaultDetectorTest : public ::testing::Test {
 public:
  EncoderFaultDetectorTest()
      : config_(aos::configuration::ReadConfig(
            "frc971/control_loops/drivetrain/simulation_config.json")),
        event_loop_factory_(&config_.message()),
        event_loop_(event_loop_factory_.MakeEventLoop(
            "drivetrain_encoder_fault_detector", nullptr)),
        drivetrain_status_fetcher_(
            event_loop_->MakeFetcher<frc971::control_loops::drivetrain::Status>(
                "/drivetrain")),
        drivetrain_position_sender_(
            event_loop_->MakeSender<drivetrain::Position>("/drivetrain")),
        drivetrain_can_position_sender_(
            event_loop_->MakeSender<drivetrain::CANPosition>("/drivetrain")),
        fault_detector_(event_loop_.get()) {}
  void ResetEncoders() {
    left_encoder_ = 0.0;
    right_encoder_ = 0.0;
    right_falcons_ = {0.0, 0.0};
    left_falcons_ = {0.0, 0.0};
  }
  aos::FlatbufferDetachedBuffer<control_loops::drivetrain::Position>
  ConstructPositionFlatBuffer(double left_encoder, double right_encoder) {
    return aos::JsonToFlatbuffer<control_loops::drivetrain::Position>(
        absl::StrFormat("{ \"left_encoder\": %f, \"right_encoder\": %f }",
                        left_encoder, right_encoder));
  }

  aos::FlatbufferDetachedBuffer<control_loops::drivetrain::CANPosition>
  ConstructCANPositionFlatBuffer(const std::vector<double> &left_falcons,
                                 const std::vector<double> &right_falcons) {
    if (left_falcons.size() == right_falcons.size()) {
      const size_t num_falcons = left_falcons.size();
      std::string json = "{ \"left_falcons\":[";

      for (size_t i = 0; i < num_falcons; ++i) {
        json += absl::StrFormat("{ \"position\": %f }", left_falcons[i]);
        if (i + 1 < num_falcons) {
          json += ", ";
        }
      }

      json += "], \"right_falcons\":[";

      for (size_t i = 0; i < num_falcons; ++i) {
        json += absl::StrFormat("{ \"position\": %f }", right_falcons[i]);
        if (i + 1 < num_falcons) {
          json += ", ";
        }
      }

      json += "]}";
      return aos::JsonToFlatbuffer<control_loops::drivetrain::CANPosition>(
          json);
    }
    LOG(FATAL) << "You must provide two falcon arrays of equal length";
    return aos::JsonToFlatbuffer<control_loops::drivetrain::CANPosition>("");
  }

  void SendPositionMessages() {
    drivetrain_position_sender_.CheckOk(drivetrain_position_sender_.Send(
        ConstructPositionFlatBuffer(left_encoder_, right_encoder_)));
    drivetrain_can_position_sender_.CheckOk(
        drivetrain_can_position_sender_.Send(
            ConstructCANPositionFlatBuffer(left_falcons_, right_falcons_)));
  }

 protected:
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory event_loop_factory_;
  std::unique_ptr<aos::EventLoop> event_loop_;

  aos::Fetcher<frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  aos::Sender<control_loops::drivetrain::Position> drivetrain_position_sender_;
  aos::Sender<control_loops::drivetrain::CANPosition>
      drivetrain_can_position_sender_;

  DrivetrainEncoderFaultDetector<2> fault_detector_;
  double left_encoder_ = 0.0;
  double right_encoder_ = 0.0;
  std::vector<double> right_falcons_;
  std::vector<double> left_falcons_;
};

// Test simulates if drivetrain encoders are idle
TEST_F(EncoderFaultDetectorTest, Idle) {
  ResetEncoders();

  event_loop_->AddTimer([this]() { SendPositionMessages(); })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(1));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

// Test simulates if drivetrain encoders are increasing
TEST_F(EncoderFaultDetectorTest, Increasing) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        left_encoder_ += 0.1;
        right_encoder_ += 0.1;
        for (double &falcon : left_falcons_) {
          falcon += 0.1;
        }
        for (double &falcon : right_falcons_) {
          falcon += 0.1;
        }
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

// Test simulates if drivetrain encoders are decreasing
TEST_F(EncoderFaultDetectorTest, Decreasing) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        left_encoder_ -= 0.1;
        right_encoder_ -= 0.1;
        for (double &falcon : left_falcons_) {
          falcon -= 0.1;
        }
        for (double &falcon : right_falcons_) {
          falcon -= 0.1;
        }
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

// Test simulates if only the right drivetrain encoders are increasing
TEST_F(EncoderFaultDetectorTest, OnlyIncreaseRightSide) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        right_encoder_ += 0.1;
        for (double &falcon : right_falcons_) {
          falcon += 0.1;
        }
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

// Test simulates if only the left drivetrain encoders are increasing
TEST_F(EncoderFaultDetectorTest, OnlyIncreaseLeftSide) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        left_encoder_ += 0.1;
        for (double &falcon : left_falcons_) {
          falcon += 0.1;
        }
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

// Test simulates if only the right drivetrain encoders are decreasing
TEST_F(EncoderFaultDetectorTest, OnlyDecreaseRightSide) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        right_encoder_ -= 0.1;
        for (double &falcon : right_falcons_) {
          falcon -= 0.1;
        }
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}
// Test simulates if only the left drivetrain encoders are decreasing
TEST_F(EncoderFaultDetectorTest, OnlyDecreaseLeftSide) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        left_encoder_ -= 0.1;
        for (double &falcon : left_falcons_) {
          falcon -= 0.1;
        }
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

// Test simulates that if there is no data for one second that their will be no
// faults
TEST_F(EncoderFaultDetectorTest, NoDataForOneSecond) {
  ResetEncoders();

  SendPositionMessages();

  event_loop_factory_.RunFor(std::chrono::seconds(1));

  left_encoder_ = 1.0;
  right_encoder_ = 2.0;
  for (double &falcon : left_falcons_) {
    falcon = 3.0;
  }
  for (double &falcon : right_falcons_) {
    falcon = 4.0;
  }

  SendPositionMessages();

  event_loop_factory_.RunFor(std::chrono::seconds(1));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

// Test simulates that only the left encoder is increasing
TEST_F(EncoderFaultDetectorTest, LeftEncoderFaulted) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        left_encoder_ += 0.1;
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      true,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}
// Test simulates that only the right encoder is increasing
TEST_F(EncoderFaultDetectorTest, RightEncoderFaulted) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        right_encoder_ += 0.1;
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      true,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

// Test simulates that only the left falcons are increasing
TEST_F(EncoderFaultDetectorTest, LeftFalconsFaulted) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        for (double &falcon : left_falcons_) {
          falcon += 0.1;
        }
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      true,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

// Test simulates that only the right falcons are increasing
TEST_F(EncoderFaultDetectorTest, RightFalconsFaulted) {
  ResetEncoders();

  event_loop_
      ->AddTimer([this]() {
        SendPositionMessages();
        for (double &falcon : right_falcons_) {
          falcon += 0.1;
        }
      })
      ->Schedule(event_loop_->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop_factory_.RunFor(std::chrono::seconds(5));

  CHECK(drivetrain_status_fetcher_.Fetch());

  EXPECT_EQ(
      true,
      drivetrain_status_fetcher_->encoder_faults()->right_faulted()->faulted());
  EXPECT_EQ(
      false,
      drivetrain_status_fetcher_->encoder_faults()->left_faulted()->faulted());
}

}  // namespace frc971::control_loops::drivetrain::testing