#include "frc971/imu_fdcan/dual_imu_blender_lib.h"

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"
#include "frc971/imu_fdcan/dual_imu_blender_lib.h"
#include "frc971/imu_fdcan/dual_imu_blender_status_generated.h"
#include "frc971/imu_fdcan/dual_imu_generated.h"
#include "frc971/imu_fdcan/dual_imu_static.h"

class DualImuBlenderTest : public ::testing::Test {
 public:
  DualImuBlenderTest()
      : config_(aos::configuration::ReadConfig(
            "frc971/imu_fdcan/dual_imu_test_config.json")),
        event_loop_factory_(&config_.message()),
        dual_imu_blender_event_loop_(
            event_loop_factory_.MakeEventLoop("dual_imu_blender")),
        dual_imu_event_loop_(event_loop_factory_.MakeEventLoop("dual_imu")),
        imu_values_batch_fetcher_(
            dual_imu_event_loop_->MakeFetcher<frc971::IMUValuesBatch>(
                "/localizer")),
        dual_imu_blender_status_fetcher_(
            dual_imu_blender_event_loop_
                ->MakeFetcher<frc971::imu::DualImuBlenderStatus>(
                    absl::GetFlag(FLAGS_use_orin1) ? "/orin1" : "/imu")),
        dual_imu_sender_(
            dual_imu_event_loop_->MakeSender<frc971::imu::DualImuStatic>(
                absl::GetFlag(FLAGS_use_orin1) ? "/orin1" : "/imu")),
        dual_imu_blender_(dual_imu_blender_event_loop_.get()) {}

  void CheckImuType(frc971::imu::ImuType type) {
    dual_imu_blender_status_fetcher_.Fetch();
    EXPECT_EQ(dual_imu_blender_status_fetcher_->gyro_x(), type);
    EXPECT_EQ(dual_imu_blender_status_fetcher_->gyro_y(), type);
    EXPECT_EQ(dual_imu_blender_status_fetcher_->gyro_z(), type);
    EXPECT_EQ(dual_imu_blender_status_fetcher_->accelerometer_x(), type);
    EXPECT_EQ(dual_imu_blender_status_fetcher_->accelerometer_y(), type);
    EXPECT_EQ(dual_imu_blender_status_fetcher_->accelerometer_z(), type);
  }

 protected:
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory event_loop_factory_;

  std::unique_ptr<aos::EventLoop> dual_imu_blender_event_loop_;
  std::unique_ptr<aos::EventLoop> dual_imu_event_loop_;

  aos::Fetcher<frc971::IMUValuesBatch> imu_values_batch_fetcher_;
  aos::Fetcher<frc971::imu::DualImuBlenderStatus>
      dual_imu_blender_status_fetcher_;

  aos::Sender<frc971::imu::DualImuStatic> dual_imu_sender_;

  frc971::imu_fdcan::DualImuBlender dual_imu_blender_;
};

// Sanity check that some sane values in are the same values out
TEST_F(DualImuBlenderTest, SanityCheck) {
  dual_imu_blender_event_loop_->OnRun([this] {
    aos::Sender<frc971::imu::DualImuStatic>::StaticBuilder dual_imu_builder =
        dual_imu_sender_.MakeStaticBuilder();

    frc971::imu::SingleImuStatic *murata = dual_imu_builder->add_murata();

    auto *murata_chip_states = murata->add_chip_states();
    frc971::imu::ChipStateStatic *murata_uno_chip_state =
        murata_chip_states->emplace_back();
    frc971::imu::ChipStateStatic *murata_due_chip_state =
        murata_chip_states->emplace_back();

    frc971::imu::SingleImuStatic *tdk = dual_imu_builder->add_tdk();

    dual_imu_builder->set_board_timestamp_us(0);
    dual_imu_builder->set_kernel_timestamp(0);

    tdk->set_gyro_x(0.3);
    tdk->set_gyro_y(0.2);
    tdk->set_gyro_z(0.2);

    murata->set_gyro_x(0.351);
    murata->set_gyro_y(0.284);
    murata->set_gyro_z(0.293);

    tdk->set_accelerometer_x(1.5);
    tdk->set_accelerometer_y(1.5);
    tdk->set_accelerometer_z(1.5);

    murata->set_accelerometer_x(1.58);
    murata->set_accelerometer_y(1.51);
    murata->set_accelerometer_z(1.52);

    murata_uno_chip_state->set_temperature(20);
    murata_due_chip_state->set_temperature(10);

    dual_imu_builder.CheckOk(dual_imu_builder.Send());
  });

  event_loop_factory_.RunFor(std::chrono::milliseconds(200));

  ASSERT_TRUE(imu_values_batch_fetcher_.Fetch());
  ASSERT_TRUE(dual_imu_blender_status_fetcher_.Fetch());

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_x(), 0.351,
              0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_y(), 0.284,
              0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_z(), 0.293,
              0.0001);

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_x(),
              1.58, 0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_y(),
              1.51, 0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_z(),
              1.52, 0.0001);

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->temperature(), 20,
              0.0001);

  CheckImuType(frc971::imu::ImuType::MURATA);
}

TEST_F(DualImuBlenderTest, Saturation) {
  dual_imu_blender_event_loop_->OnRun([this] {
    aos::Sender<frc971::imu::DualImuStatic>::StaticBuilder dual_imu_builder =
        dual_imu_sender_.MakeStaticBuilder();

    frc971::imu::SingleImuStatic *murata = dual_imu_builder->add_murata();

    auto *murata_chip_states = murata->add_chip_states();
    frc971::imu::ChipStateStatic *murata_uno_chip_state =
        murata_chip_states->emplace_back();

    frc971::imu::SingleImuStatic *tdk = dual_imu_builder->add_tdk();

    dual_imu_builder->set_board_timestamp_us(0);
    dual_imu_builder->set_kernel_timestamp(0);

    tdk->set_gyro_x(0.7);
    tdk->set_gyro_y(0.7);
    tdk->set_gyro_z(0.7);

    murata->set_gyro_x(0.71);
    murata->set_gyro_y(0.79);
    murata->set_gyro_z(0.78);

    tdk->set_accelerometer_x(1.0);
    tdk->set_accelerometer_y(1.0);
    tdk->set_accelerometer_z(1.0);

    murata->set_accelerometer_x(1.3);
    murata->set_accelerometer_y(1.1);
    murata->set_accelerometer_z(1.1);

    murata_uno_chip_state->set_temperature(20);

    dual_imu_builder.CheckOk(dual_imu_builder.Send());
  });

  bool enable_tdk_sender = false;
  dual_imu_blender_event_loop_->AddPhasedLoop(
      [this, &enable_tdk_sender](int) {
        if (!enable_tdk_sender) return;
        aos::Sender<frc971::imu::DualImuStatic>::StaticBuilder
            dual_imu_builder = dual_imu_sender_.MakeStaticBuilder();

        frc971::imu::SingleImuStatic *murata = dual_imu_builder->add_murata();

        auto *murata_chip_states = murata->add_chip_states();
        frc971::imu::ChipStateStatic *murata_uno_chip_state =
            murata_chip_states->emplace_back();

        frc971::imu::SingleImuStatic *tdk = dual_imu_builder->add_tdk();

        dual_imu_builder->set_board_timestamp_us(1);
        dual_imu_builder->set_kernel_timestamp(1);

        tdk->set_gyro_x(6.0);
        tdk->set_gyro_y(6.0);
        tdk->set_gyro_z(6.0);

        murata->set_gyro_x(5.2);
        murata->set_gyro_y(5.2);
        murata->set_gyro_z(5.2);

        tdk->set_accelerometer_x(6.2);
        tdk->set_accelerometer_y(6.3);
        tdk->set_accelerometer_z(6.5);

        murata->set_accelerometer_x(5.5);
        murata->set_accelerometer_y(5.5);
        murata->set_accelerometer_z(5.5);

        murata_uno_chip_state->set_temperature(20);

        dual_imu_builder.CheckOk(dual_imu_builder.Send());
      },
      std::chrono::milliseconds(1));

  bool send_negative = false;
  dual_imu_blender_event_loop_->AddPhasedLoop(
      [this, &send_negative](int) {
        if (!send_negative) return;

        aos::Sender<frc971::imu::DualImuStatic>::StaticBuilder
            dual_imu_builder = dual_imu_sender_.MakeStaticBuilder();

        frc971::imu::SingleImuStatic *murata = dual_imu_builder->add_murata();

        auto *murata_chip_states = murata->add_chip_states();
        frc971::imu::ChipStateStatic *murata_uno_chip_state =
            murata_chip_states->emplace_back();

        frc971::imu::SingleImuStatic *tdk = dual_imu_builder->add_tdk();

        dual_imu_builder->set_board_timestamp_us(1);
        dual_imu_builder->set_kernel_timestamp(1);

        tdk->set_gyro_x(-6.0);
        tdk->set_gyro_y(-6.0);
        tdk->set_gyro_z(-6.0);

        murata->set_gyro_x(-5.2);
        murata->set_gyro_y(-5.2);
        murata->set_gyro_z(-5.2);

        tdk->set_accelerometer_x(-6.2);
        tdk->set_accelerometer_y(-6.3);
        tdk->set_accelerometer_z(-6.5);

        murata->set_accelerometer_x(-5.5);
        murata->set_accelerometer_y(-5.5);
        murata->set_accelerometer_z(-5.5);

        murata_uno_chip_state->set_temperature(20);

        dual_imu_builder.CheckOk(dual_imu_builder.Send());
      },
      std::chrono::milliseconds(1));

  event_loop_factory_.RunFor(std::chrono::milliseconds(200));

  ASSERT_TRUE(imu_values_batch_fetcher_.Fetch());
  ASSERT_TRUE(dual_imu_blender_status_fetcher_.Fetch());

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_x(), 0.71,
              0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_y(), 0.79,
              0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_z(), 0.78,
              0.0001);

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_x(),
              1.3, 0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_y(),
              1.1, 0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_z(),
              1.1, 0.0001);

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->temperature(), 20,
              0.0001);

  CheckImuType(frc971::imu::ImuType::MURATA);

  // Make sure we switch to TDK on saturation
  enable_tdk_sender = true;

  event_loop_factory_.RunFor(std::chrono::milliseconds(200));

  ASSERT_TRUE(imu_values_batch_fetcher_.Fetch());
  ASSERT_TRUE(dual_imu_blender_status_fetcher_.Fetch());

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_x(), 6.0,
              0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_y(), 6.0,
              0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_z(), 6.0,
              0.0001);

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_x(),
              6.2, 0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_y(),
              6.3, 0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_z(),
              6.5, 0.0001);

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->temperature(), 20,
              0.0001);

  CheckImuType(frc971::imu::ImuType::TDK);

  // Check negative values as well
  enable_tdk_sender = false;
  send_negative = true;

  event_loop_factory_.RunFor(std::chrono::milliseconds(200));

  ASSERT_TRUE(imu_values_batch_fetcher_.Fetch());
  ASSERT_TRUE(dual_imu_blender_status_fetcher_.Fetch());

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_x(), -6.0,
              0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_y(), -6.0,
              0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->gyro_z(), -6.0,
              0.0001);

  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_x(),
              -6.2, 0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_y(),
              -6.3, 0.0001);
  EXPECT_NEAR(imu_values_batch_fetcher_->readings()->Get(0)->accelerometer_z(),
              -6.5, 0.0001);

  CheckImuType(frc971::imu::ImuType::TDK);
}
