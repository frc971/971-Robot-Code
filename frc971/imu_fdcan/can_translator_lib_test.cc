#include "frc971/imu_fdcan/can_translator_lib.h"

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"
#include "frc971/can_logger/can_logging_static.h"
#include "frc971/imu_fdcan/can_translator_status_generated.h"
#include "frc971/imu_fdcan/dual_imu_generated.h"

class CANTranslatorTest : public ::testing::Test {
 public:
  CANTranslatorTest()
      : config_(aos::configuration::ReadConfig(
            "frc971/imu_fdcan/dual_imu_test_config.json")),
        event_loop_factory_(&config_.message()),
        can_translator_event_loop_(
            event_loop_factory_.MakeEventLoop("can_translator")),
        can_frame_event_loop_(event_loop_factory_.MakeEventLoop("can_frame")),
        dual_imu_fetcher_(
            can_translator_event_loop_->MakeFetcher<frc971::imu::DualImu>(
                absl::GetFlag(FLAGS_use_one_orin) ? "/orin1" : "/imu")),
        can_translator_status_fetcher_(
            can_translator_event_loop_
                ->MakeFetcher<frc971::imu::CanTranslatorStatus>(
                    absl::GetFlag(FLAGS_use_one_orin) ? "/orin1" : "/imu")),
        can_frame_sender_(
            can_frame_event_loop_
                ->MakeSender<frc971::can_logger::CanFrameStatic>("/can")),
        can_translator_(can_translator_event_loop_.get(), "/can") {}

 protected:
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory event_loop_factory_;

  std::unique_ptr<aos::EventLoop> can_translator_event_loop_;
  std::unique_ptr<aos::EventLoop> can_frame_event_loop_;

  aos::Fetcher<frc971::imu::DualImu> dual_imu_fetcher_;
  aos::Fetcher<frc971::imu::CanTranslatorStatus> can_translator_status_fetcher_;

  aos::Sender<frc971::can_logger::CanFrameStatic> can_frame_sender_;

  frc971::imu_fdcan::CANTranslator can_translator_;
};

TEST_F(CANTranslatorTest, CheckValidFrame) {
  event_loop_factory_.GetNodeEventLoopFactory(can_frame_event_loop_->node())
      ->SetRealtimeOffset(
          aos::monotonic_clock::epoch() + std::chrono::seconds(0),
          aos::realtime_clock::epoch() + std::chrono::seconds(100));
  can_frame_event_loop_->OnRun([this] {
    std::array<uint8_t, 64> full_frame{
        226, 100, 108, 8,   152, 40,  202, 121, 202, 121, 202, 121, 85,
        85,  81,  189, 0,   0,   8,   189, 85,  213, 127, 191, 12,  189,
        34,  187, 255, 219, 220, 59,  147, 173, 5,   61,  88,  68,  205,
        188, 230, 92,  24,  189, 235, 1,   127, 191, 210, 7,   34,  54,
        86,  103, 133, 186, 100, 205, 101, 185, 29,  26,  26,  0};
    for (size_t i = 0; i < 8; ++i) {
      aos::Sender<frc971::can_logger::CanFrameStatic>::StaticBuilder
          can_frame_builder = can_frame_sender_.MakeStaticBuilder();

      can_frame_builder->set_can_id(i + 1);
      can_frame_builder->set_realtime_timestamp_ns(100e9 + 971);
      auto can_data = can_frame_builder->add_data();
      CHECK(can_data->FromData(full_frame.data() + i * 8, 8));

      can_frame_builder.CheckOk(can_frame_builder.Send());
    }
  });

  event_loop_factory_.RunFor(std::chrono::milliseconds(200));

  ASSERT_TRUE(can_translator_status_fetcher_.Fetch());
  ASSERT_TRUE(dual_imu_fetcher_.Fetch());

  ASSERT_FALSE(can_translator_status_fetcher_->invalid_packet_count() > 0);
  ASSERT_FALSE(can_translator_status_fetcher_->invalid_can_id_count() > 0);
  EXPECT_EQ(can_translator_status_fetcher_->valid_packet_count(), 8);

  EXPECT_EQ(dual_imu_fetcher_->board_timestamp_us(), 141321442);
  EXPECT_EQ(dual_imu_fetcher_->packet_counter(), 10392);

  EXPECT_NEAR(dual_imu_fetcher_->murata()->gyro_x(), 2.41444e-06, 0.00001);
  EXPECT_NEAR(dual_imu_fetcher_->murata()->gyro_y(), -0.00101779, 0.00001);
  EXPECT_NEAR(dual_imu_fetcher_->murata()->gyro_z(), -0.000219157, 0.00001);

  EXPECT_NEAR(dual_imu_fetcher_->murata()->accelerometer_x(), -0.025057,
              0.00001);
  EXPECT_NEAR(dual_imu_fetcher_->murata()->accelerometer_y(), -0.037198,
              0.00001);
  EXPECT_NEAR(dual_imu_fetcher_->murata()->accelerometer_z(), -0.996123,
              0.00001);

  EXPECT_EQ(dual_imu_fetcher_->murata()->chip_states()->Get(0)->counter(),
            31178);
  EXPECT_EQ(dual_imu_fetcher_->murata()->chip_states()->Get(0)->temperature(),
            26);

  EXPECT_EQ(dual_imu_fetcher_->murata()->chip_states()->Get(1)->counter(),
            31178);
  EXPECT_EQ(dual_imu_fetcher_->murata()->chip_states()->Get(1)->temperature(),
            26);

  EXPECT_NEAR(dual_imu_fetcher_->tdk()->gyro_x(), -0.00248319, 0.00001);
  EXPECT_NEAR(dual_imu_fetcher_->tdk()->gyro_y(), 0.00674009, 0.00001);
  EXPECT_NEAR(dual_imu_fetcher_->tdk()->gyro_z(), 0.0326362, 0.00001);

  EXPECT_NEAR(dual_imu_fetcher_->tdk()->accelerometer_x(), -0.0511068, 0.00001);
  EXPECT_NEAR(dual_imu_fetcher_->tdk()->accelerometer_y(), -0.0332031, 0.00001);
  EXPECT_NEAR(dual_imu_fetcher_->tdk()->accelerometer_z(), -0.999349, 0.00001);

  EXPECT_EQ(dual_imu_fetcher_->tdk()->chip_states()->Get(0)->counter(), 31178);
  EXPECT_EQ(dual_imu_fetcher_->tdk()->chip_states()->Get(0)->temperature(), 29);

  EXPECT_EQ(dual_imu_fetcher_->kernel_timestamp(), 971);
}

TEST_F(CANTranslatorTest, CheckInvalidFrame) {
  can_frame_event_loop_->OnRun([this] {
    aos::Sender<frc971::can_logger::CanFrameStatic>::StaticBuilder
        can_frame_builder = can_frame_sender_.MakeStaticBuilder();

    can_frame_builder->set_can_id(10);
    can_frame_builder->set_realtime_timestamp_ns(100);
    auto can_data = can_frame_builder->add_data();
    CHECK(can_data->reserve(sizeof(uint8_t) * 8));
    can_data->resize(8);

    can_frame_builder.CheckOk(can_frame_builder.Send());
  });

  event_loop_factory_.RunFor(std::chrono::milliseconds(200));

  ASSERT_TRUE(can_translator_status_fetcher_.Fetch());
  ASSERT_FALSE(dual_imu_fetcher_.Fetch());

  EXPECT_EQ(can_translator_status_fetcher_->invalid_packet_count(), 0);
  EXPECT_EQ(can_translator_status_fetcher_->invalid_can_id_count(), 1);
}
