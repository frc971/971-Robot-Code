#include <stdio.h>

#include <algorithm>
#include <limits>
#include <cmath>

#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "core/VL53L1X_api.h"
#include "core/VL53L1X_calibration.h"

namespace y2023 {
namespace tof_controller {

static constexpr uint kI2CBaudrate = 100000;
static constexpr uint16_t kExpectedSensorId = 0xEBAA;

static constexpr uint16_t kTimingBudgetMs = 33;  // 15 ms is fastest
static constexpr double kWatchdogTimeoutMs = 2000;
static constexpr int64_t kSensorBootTimeoutMs = 3;

static constexpr double kSensorSeparation = 0.45;
static constexpr int16_t kSensorOffsetMM = -22;

/*
// Cone base
static constexpr double kMaxObstructionWidth = 0.22 + 0.07;
// cone tip
static constexpr double kMinObstructionWidth = 0.044 + 0.07;
*/

namespace sensor1 {
static constexpr uint kPinSCL = 1;
static constexpr uint kPinSDA = 0;

static constexpr uint kPinStatusGreen = 3;
static constexpr uint kPinStatusRed = 5;
static constexpr uint kPinStatusPWM = 16;

static constexpr uint kPinInterrupt = 8;
static constexpr uint kPinShutdown = 9;

static constexpr uint kPinOutput = 2;
}  // namespace sensor1

namespace sensor2 {
static constexpr uint kPinSCL = 26;
static constexpr uint kPinSDA = 27;

static constexpr uint kPinStatusGreen = 14;
static constexpr uint kPinStatusRed = 13;
static constexpr uint kPinStatusPWM = 17;

static constexpr uint kPinInterrupt = 22;
static constexpr uint kPinShutdown = 28;

static constexpr uint kPinOutput = 4;
}  // namespace sensor2

class SignalWriter {
 public:
  static constexpr double kScaledRangeLow = 0.1;
  static constexpr double kScaledRangeHigh = 0.9;

  // PWM counts to this before wrapping
  static constexpr uint16_t kPWMTop = 62499;
  static constexpr int kPWMFreqHz = 200;

  SignalWriter(uint pin) : pin_(pin) {
    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_enabled(slice_num, true);

    // TODO(Ravago): Verify that this still works the same as the imu board

    /* frequency_pwm = f_sys / ((TOP + 1) * (CSR_PHASE_CORRECT + 1) * (DIV_INT +
     * DIV_FRAC / 16))
     *
     * f_sys = 125 mhz
     * CSR_PHASE_CORRECT = 0; no phase correct
     * TARGET_FREQ = 200 hz
     *
     * 125 mhz / x = 200 hz * 62500
     *
     * TOP = 62499
     * DIV_INT = 10
     */

    float divisor = clock_get_hz(clk_sys) / (kPWMTop + 1) / kPWMFreqHz;

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
    pwm_config_set_clkdiv(&cfg, divisor);
    pwm_config_set_wrap(&cfg, kPWMTop);

    pwm_init(slice_num, &cfg, true);

    Write();
  }

  void SetValue(double value) {
    value_ = std::clamp(value, 0.0, 1.0);
    Write();
  }

  void SetEnabled(bool enabled) {
    enabled_ = enabled;
    Write();
  }

 private:
  void Write() {
    double scaled_value =
        (value_ * (kScaledRangeHigh - kScaledRangeLow) + kScaledRangeLow);

    uint16_t level = scaled_value * kPWMTop;

    if (!enabled_) {
      level = 0;
    }

    pwm_set_gpio_level(pin_, level);
  }

  double value_ = 0.0;
  uint pin_;
  bool enabled_ = true;
};

class BrightnessLED {
 public:
  BrightnessLED(uint pin) : pin_(pin) {
    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_enabled(slice_num, true);

    Write();
  }

  void SetValue(double value) {
    value_ = std::clamp(value, 0.0, 1.0);
    Write();
  }

 private:
  void Write() {
    uint16_t level = value_ * std::numeric_limits<uint16_t>::max();
    pwm_set_gpio_level(pin_, level);
  }

  double value_ = 0;
  uint pin_;
};

class StatusLED {
 public:
  StatusLED(uint pin_green, uint pin_red)
      : pin_green_(pin_green), pin_red_(pin_red) {
    gpio_init(pin_green);
    gpio_init(pin_red);
    gpio_set_dir(pin_green, GPIO_OUT);
    gpio_set_dir(pin_red, GPIO_OUT);

    SetError();
  }

  void SetOk() { Set(true); }

  void SetError() { Set(false); }

  void Set(bool is_good) {
    status_ok_ = is_good;
    Write();
  }

  void Toggle() {
    status_ok_ = !status_ok_;
    Write();
  }

  bool get_status() { return status_ok_; }

 private:
  void Write() {
    if (status_ok_) {
      gpio_put(pin_green_, true);
      gpio_put(pin_red_, false);
    } else {
      gpio_put(pin_green_, false);
      gpio_put(pin_red_, true);
    }
  }

  uint pin_green_, pin_red_;
  bool status_ok_ = false;
};

class VL53L1X {
 public:
  VL53L1X(int id, StatusLED status_light, BrightnessLED brightness,
          uint interrupt_pin, uint shutdown_pin)
      : id_(id),
        status_light_(status_light),
        brightness_(brightness),
        interrupt_pin_(interrupt_pin),
        shutdown_pin_(shutdown_pin) {
    gpio_init(interrupt_pin);
    gpio_set_dir(interrupt_pin, GPIO_IN);

    gpio_init(shutdown_pin);
    gpio_set_dir(shutdown_pin, GPIO_OUT);
    gpio_put(shutdown_pin, true);

    InitSensor();
  }

  void WaitForData() {
    uint8_t data_ready = false;
    while (!data_ready) {
      CheckOk(VL53L1X_CheckForDataReady(id_, &data_ready));

      if (errors > 0) {
        // don't block the main loop if we're in an errored state
        break;
      }

      sleep_us(1);
    }
  }

  VL53L1X_Result_t Read() {
    VL53L1X_Result_t result;

    CheckOk(VL53L1X_GetResult(id_, &result));

    double meters = static_cast<double>(result.Distance) * 0.001;
    brightness_.SetValue(meters / kSensorSeparation);

    return result;
  }

  void Stop() {
    CheckOk(VL53L1X_StopRanging(id_));

    // StopRanging command is checked when the interrupt is cleared
    CheckOk(VL53L1X_ClearInterrupt(id_));

    // wait for it to actually turn off the light so it doesn't interfere with
    // the other sensor
    sleep_ms(3);
  }

  void Start() { CheckOk(VL53L1X_StartRanging(id_)); }

  // Try to reinitialize the sensor if it is in a bad state (eg. lost power
  // but is now reconnected)
  void MaybeAttemptRecovery() {
    if (errors > 0 && consecutive_errors == 0) {
      // it's probably back
      printf("Attempting to recover from an errored state on sensor %d.\n",
             id_);
      printf("Previously had %d total errors\n", errors);

      // Reboot to put it in a known state.
      gpio_put(shutdown_pin_, false);
      sleep_ms(500);
      gpio_put(shutdown_pin_, true);

      InitSensor();
    }
  }

  int errors = 0;
  int consecutive_errors = 0;

 private:
  void InitSensor() {
    printf("Waiting for sensor %d to boot.\n", id_);

    absolute_time_t start_time = get_absolute_time();
    uint8_t boot_state = 0;
    while (boot_state == 0) {
      CheckOk(VL53L1X_BootState(id_, &boot_state));

      int64_t diff = absolute_time_diff_us(start_time, get_absolute_time());

      if (diff > (kSensorBootTimeoutMs * 1000)) {
        printf("Timed out after %lld us\n", diff);
        return;
      }

      sleep_us(1);
    }
    printf("Bootstate: %d\n", boot_state);

    int status = 0;

    printf("Getting sensor id\n");
    // Validate sensor model ID and Type
    uint16_t sensor_id;
    status += VL53L1X_GetSensorId(id_, &sensor_id);
    if (sensor_id != kExpectedSensorId) {  // Bad connection, wrong chip, etc
      printf("Bad sensor id %x, continuing anyways\n", sensor_id);
      status_light_.SetError();
    }
    printf("Got sensor id: %x\n", sensor_id);

    // SensorInit will cause the processor to hang and hit the watchdog if the
    // sensor is in a bad state
    status += VL53L1X_SensorInit(id_);

    status += VL53L1X_SetDistanceMode(id_, 1);  // Set distance mode to short
    status += VL53L1X_SetTimingBudgetInMs(id_, kTimingBudgetMs);
    status += VL53L1X_SetInterMeasurementInMs(id_, kTimingBudgetMs);
    status += VL53L1X_SetOffset(id_, kSensorOffsetMM);
    status += VL53L1X_StartRanging(id_);

    if (status == 0) {
      printf("Successfully configured sensor %d\n", id_);

      status_light_.SetOk();
      errors = 0;
      consecutive_errors = 0;
    } else {
      printf("Failed to configure sensor %d\n", id_);
    }
  }

  void CheckOk(VL53L1X_ERROR error) {
    if (error == 0) {
      consecutive_errors = 0;
      return;
    }

    consecutive_errors++;
    errors++;

    status_light_.SetError();
  }

  // id of the sensor ie. sensor 1 or sensor 2
  int id_;
  StatusLED status_light_;
  BrightnessLED brightness_;
  uint interrupt_pin_;
  uint shutdown_pin_;
};

int main() {
  stdio_init_all();

  if (watchdog_caused_reboot()) {
    printf("Rebooted by Watchdog!\n");
  } else {
    printf("Clean boot\n");
  }

  // Enable the watchdog, requiring the watchdog to be updated every 100ms or
  // the chip will reboot second arg is pause on debug which means the watchdog
  // will pause when stepping through code
  watchdog_enable(kWatchdogTimeoutMs, 1);

  i2c_init(i2c0, kI2CBaudrate);
  gpio_set_function(sensor1::kPinSCL, GPIO_FUNC_I2C);
  gpio_set_function(sensor1::kPinSDA, GPIO_FUNC_I2C);

  i2c_init(i2c1, kI2CBaudrate);
  gpio_set_function(sensor2::kPinSCL, GPIO_FUNC_I2C);
  gpio_set_function(sensor2::kPinSDA, GPIO_FUNC_I2C);

  StatusLED status_light1(sensor1::kPinStatusGreen, sensor1::kPinStatusRed);
  StatusLED status_light2(sensor2::kPinStatusGreen, sensor2::kPinStatusRed);
  BrightnessLED dist_light1(sensor1::kPinStatusPWM);
  BrightnessLED dist_light2(sensor2::kPinStatusPWM);

  BrightnessLED output_indicator(PICO_DEFAULT_LED_PIN);
  SignalWriter output_writer(sensor1::kPinOutput);

  printf("Initializing\n");

  VL53L1X sensor1(1, status_light1, dist_light1, sensor1::kPinInterrupt,
                  sensor1::kPinShutdown);
  VL53L1X sensor2(2, status_light2, dist_light2, sensor2::kPinInterrupt,
                  sensor2::kPinShutdown);

  /* temporary*/ int n = 0;
  double mean = 0;
  double M2 = 0;

  absolute_time_t last_reading = get_absolute_time();

  // Measure and print continuously
  while (1) {
    sensor1.Start();
    sensor1.WaitForData();
    VL53L1X_Result_t result1 = sensor1.Read();
    sensor1.Stop();

    sensor2.Start();
    sensor2.WaitForData();
    VL53L1X_Result_t result2 = sensor2.Read();
    sensor2.Stop();

    double dist1 = static_cast<double>(result1.Distance) * 0.001;
    double dist2 = static_cast<double>(result2.Distance) * 0.001;

    // Estimates the center of the obstruction
    // where 0 is at sensor 1 (left)
    // and kSensorSeparation is at sensor 2 (right)
    double width_of_obstruction = kSensorSeparation - (dist1 + dist2);

    double left_estimate = (dist1 + (width_of_obstruction / 2));
    double right_estimate =
        (kSensorSeparation - dist2) - (width_of_obstruction / 2);

    double averaged_estimate = (left_estimate + right_estimate) / 2;

    bool data_good = sensor1.errors == 0 && result1.Status == 0 &&
                     sensor2.errors == 0 && result2.Status == 0 &&
                     width_of_obstruction > 0;

    output_writer.SetEnabled(data_good);
    output_writer.SetValue(averaged_estimate);
    output_indicator.SetValue(data_good? averaged_estimate: 0);

    /*Temporary */ if (data_good) {
      n += 1;
      double x = dist1 * 1000;
      double delta = x - mean;
      mean += delta / n;
      M2 += delta * (x - mean);
      printf("Std dev: %f mm\n", sqrt(M2 / (n - 1)));
    } else {
      n = 0;
      mean = 0;
      M2 = 0;
    }

    watchdog_update();
    absolute_time_t now = get_absolute_time();
    int64_t diff = absolute_time_diff_us(last_reading, now);
    last_reading = now;
    double period_ms = static_cast<double>(diff) * 0.001;

    printf(
        "Status = %2d, dist = %5d mm, Ambient = %3d, Signal = %5d, #ofSpads "
        "= %3d\nStatus = %2d, "
        "dist = %5d mm, "
        "Ambient = %3d, Signal = %5d, "
        "#ofSpads = %3d\nPeriod: %f Errors: %d %d %d %d\nx: %f, width: %f "
        "data: %s\n",
        result1.Status, result1.Distance, result1.Ambient, result1.SigPerSPAD,
        result1.NumSPADs, result2.Status, result2.Distance, result2.Ambient,
        result2.SigPerSPAD, result2.NumSPADs, period_ms, sensor1.errors,
        sensor2.errors, sensor1.consecutive_errors, sensor2.consecutive_errors,
        averaged_estimate, width_of_obstruction, data_good ? "good" : "bad");

    // Try to reinitialize the sensor if it is in a bad state (eg. lost power
    // but is now reconnected)
    sensor1.MaybeAttemptRecovery();
    sensor2.MaybeAttemptRecovery();

    // allow the user to enter the bootloader without removing power or having
    // to install a reset button
    char user_input = getchar_timeout_us(0);
    if (user_input == 'q') {
      printf("Going down! entering bootloader\n");
      reset_usb_boot(0, 0);
    }

    // Calibration mode
    if (user_input == 'c') {
      printf(
          "Entering calibration mode\n"
          "Please place 17%% gray target 100 mm from sensor 1 and press c "
          "again to continue.\n");
      while (true) {
        user_input = getchar_timeout_us(0);
        watchdog_update();
        if (user_input == 'c') break;
      }

      printf("Taking 50 measurements\n");
      int16_t offset = 0;
      VL53L1X_CalibrateOffset(1, 100, &offset);
      printf("Got an offset of %d\n", offset);

      sleep_ms(1000);
    }
  }
}

}  // namespace tof_controller
}  // namespace y2023

int main() { y2023::tof_controller::main(); }
