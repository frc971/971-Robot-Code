#ifndef Y2022_VISION_GPIO_H_
#define Y2022_VISION_GPIO_H_

/* Modified from: https://elinux.org/RPi_GPIO_Code_Samples
 * Raspberry Pi GPIO example using sysfs interface.
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "aos/init.h"

namespace y2022 {
namespace vision {

using namespace frc971::vision;

// Physical pin 19 maps to sysfs pin 10
// This pin is MOSI, being used to control the LED Disable
static constexpr int GPIO_PIN_MOSI_DISABLE = 10;

// Physical pin 23 maps to sysfs pin 11
// This pin is SCLK, and is used to talk to the IMU
// To drive the lights, we have to set this pin to an input
static constexpr int GPIO_PIN_SCLK_IMU = 11;

// Physical pin 33 maps to sysfs pin 13
// This pin is SCK, being used to control the LED PWM
static constexpr int GPIO_PIN_SCK_PWM = 13;

static constexpr int BUFFER_MAX = 3;
static constexpr int VALUE_MAX = 30;

// Is GPIO an input (read) or output (write)
static constexpr int kGPIOIn = 0;
static constexpr int kGPIOOut = 1;

// Pin voltage level low or high
static constexpr int kGPIOLow = 0;
static constexpr int kGPIOHigh = 1;

class GPIOControl {
 public:
  GPIOControl(int pin, int direction, int default_value = kGPIOLow)
      : pin_(pin), direction_(direction), default_value_(default_value) {
    // Set up the pin
    GPIOExport();
    GPIODirection();

    // Get the file descriptor used to read / write
    gpio_rw_fd_ = GPIOGetFileDesc();

    // If it's an output, set it in the appropriate default state
    if (direction_ == kGPIOOut) {
      GPIOWrite(default_value_);
    }
  }

  ~GPIOControl() {
    if (direction_ == kGPIOOut) {
      // Set pin to "default" value (e.g., turn off before closing)
      GPIOWrite(default_value_);
    }

    close(gpio_rw_fd_);
    GPIOUnexport();
  }

  //
  // Set up pin for active use ("export" it)
  // Keeping this static method that allows exporting a specific pin #
  //
  static int GPIOExport(int pin) {
    VLOG(2) << "GPIOExport of pin " << pin;

    char buffer[BUFFER_MAX];
    size_t bytes_to_write, bytes_written;

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (-1 == fd) {
      LOG(INFO) << "Failed to open export for writing!";
      return (-1);
    }

    bytes_to_write = snprintf(buffer, BUFFER_MAX, "%d", pin);
    bytes_written = write(fd, buffer, bytes_to_write);
    if (bytes_to_write != bytes_written) {
      LOG(INFO) << "Issue exporting GPIO pin " << pin
                << ".  May be OK if pin already exported";
    }

    close(fd);

    // Adding this sleep, since we get some sort of race condition
    // if we try to act on this too soon after export
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return 0;
  }

  // For calling pin defined by this class
  int GPIOExport() { return GPIOExport(pin_); }

  //
  // Remove pin from active use ("unexport" it)
  // Keeping this static method that allows unexporting of a specific pin #
  //
  static int GPIOUnexport(int pin) {
    VLOG(2) << "GPIOUnexport of pin " << pin;

    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (-1 == fd) {
      LOG(INFO) << "Failed to open unexport for writing!";
      return (-1);
    }

    char buffer[BUFFER_MAX];
    ssize_t bytes_to_write, bytes_written;
    bytes_to_write = snprintf(buffer, BUFFER_MAX, "%d", pin);
    bytes_written = write(fd, buffer, bytes_to_write);
    if (bytes_to_write != bytes_written) {
      LOG(INFO) << "Issue unexporting GPIO pin " << pin
                << ".  May be OK if pin already unexported";
    }

    close(fd);
    return 0;
  }

  int GPIOUnexport() { return GPIOUnexport(pin_); }

  //
  // Set input / output direction of a pin
  // Keeping this static method that allows setting direction of a specific pin
  //
  static int GPIODirection(int pin, int dir) {
    VLOG(2) << "Setting GPIODirection for pin " << pin << " to " << dir;
    constexpr int DIRECTION_MAX = 50;
    char path[DIRECTION_MAX];

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
    int fd = open(path, O_WRONLY);
    if (-1 == fd) {
      LOG(ERROR) << "Failed to open gpio direction for writing!\nPath = "
                 << path;
      return (-1);
    }

    if (-1 == write(fd, ((kGPIOIn == dir) ? "in" : "out"),
                    ((kGPIOIn == dir) ? 2 : 3))) {
      LOG(ERROR) << "Failed to set direction!";
      return (-1);
    }

    close(fd);
    // TODO: See if we can remove this sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return 0;
  }

  // Call for internal use of pin_ and direction_
  int GPIODirection() { return GPIODirection(pin_, direction_); }

  //
  // Read pin
  // Keeping this static method that allows reading of a specific pin with no
  // file descriptor open
  //
  static int GPIORead(int pin, int fd = -1) {
    char value_str[3];
    bool need_to_close_fd = false;

    if (fd == -1) {
      char path[VALUE_MAX];

      snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
      fd = open(path, O_RDONLY);
      if (-1 == fd) {
        LOG(ERROR) << "Failed to open gpio value for reading on pin " << pin;
        return (-1);
      }
      need_to_close_fd = true;
    }

    if (-1 == read(fd, value_str, 3)) {
      LOG(ERROR) << "Failed to read value on pin " << pin;
      return (-1);
    }

    if (need_to_close_fd) {
      close(fd);
    }

    return (atoi(value_str));
  }

  int GPIORead() { return GPIORead(pin_, gpio_rw_fd_); }

  //
  // Write to pin
  // Keeping this static method that allows writing to a specific pin with no
  // file descriptor open
  //
  static int GPIOWrite(int pin, int value, int fd = -1) {
    static const char s_values_str[] = "01";
    bool need_to_close_fd = false;

    if (fd == -1) {
      char path[VALUE_MAX];
      snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
      fd = open(path, O_WRONLY);
      if (-1 == fd) {
        LOG(ERROR) << "Failed to open gpio pin " << pin
                   << " for reading / writing";
        return (-1);
      }
      LOG(INFO) << "Opened fd as " << fd;
      need_to_close_fd = true;
    }

    if (1 != write(fd, &s_values_str[(kGPIOLow == value) ? 0 : 1], 1)) {
      LOG(ERROR) << "Failed to write value " << value << " to pin " << pin;
      return (-1);
    }

    if (need_to_close_fd) {
      close(fd);
    }
    return 0;
  }

  int GPIOWrite(int value) { return GPIOWrite(pin_, value, gpio_rw_fd_); }

  int GPIOGetFileDesc() {
    char path[VALUE_MAX];
    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin_);
    int fd = open(path, O_WRONLY);
    if (-1 == fd) {
      LOG(ERROR) << "Failed to open gpio pin " << pin_
                 << " for reading / writing";
      return (-1);
    }
    return fd;
  }

  // pin # to read / write to
  int pin_;
  // direction for reading / writing
  int direction_;
  // A default ("safe") value to set the pin to on start / finish
  int default_value_;
  // file descriptor for reading / writing
  int gpio_rw_fd_;
};

// Control GPIO pin using HW Control
// A bit hacky-- uses system call to gpio library

class GPIOPWMControl {
 public:
  GPIOPWMControl(int pin, double duty_cycle)
      : pin_(pin), gpio_control_(GPIOControl(pin_, kGPIOOut, kGPIOLow)) {
    VLOG(2) << "Using gpio command-based PWM control";
    // Set pin to PWM mode
    std::string cmd = "gpio -g mode " + std::to_string(pin_) + " pwm";
    int ret = std::system(cmd.c_str());
    CHECK_EQ(ret, 0) << "cmd: " + cmd + " failed with return value " << ret;

    // Set PWM mode in Mark-Space mode (duty cycle is on, then off)
    cmd = "gpio pwm-ms";
    ret = std::system(cmd.c_str());
    CHECK_EQ(ret, 0) << "cmd: " + cmd + " failed with return value " << ret;

    // Our PWM clock is going at 19.2 MHz
    // With a clock divisor of 192, we get a clock tick of 100kHz
    // or 10 us / tick
    cmd = "gpio pwmc 192";
    ret = std::system(cmd.c_str());
    CHECK_EQ(ret, 0) << "cmd: " + cmd + " failed with return value " << ret;

    // With a range of 100, and 10 us / tick, this gives
    // a period of 100*10us = 1ms, or 1kHz frequency
    cmd = "gpio pwmr " + std::to_string(kRange);
    ret = std::system(cmd.c_str());
    CHECK_EQ(ret, 0) << "cmd: " + cmd + " failed with return value " << ret;

    setPWMDutyCycle(duty_cycle);
  };

  ~GPIOPWMControl() { setPWMDutyCycle(0.0); }

  int setPWMDutyCycle(double duty_cycle) {
    CHECK_GE(duty_cycle, 0.0) << "Duty Cycle must be between 0 and 1";
    CHECK_LE(duty_cycle, 1.0) << "Duty Cycle must be between 0 and 1";
    int val = static_cast<int>(duty_cycle * kRange);
    std::string cmd =
        "gpio -g pwm " + std::to_string(pin_) + " " + std::to_string(val);
    int ret = std::system(cmd.c_str());
    CHECK_EQ(ret, 0) << "cmd: " + cmd + " failed with return value " << ret;

    // TODO: Maybe worth doing error handling / return
    return ret;
  }

  static constexpr int kRange = 100;
  int pin_;
  GPIOControl gpio_control_;
};

// Hack up a SW controlled PWM, in case we need it

class GPIOSWPWMControl {
 public:
  GPIOSWPWMControl(aos::ShmEventLoop *event_loop, int pin, double frequency,
                   double duty_cycle)
      : event_loop_(event_loop),
        pin_(pin),
        frequency_(frequency),
        duty_cycle_(duty_cycle),
        leds_on_(false),
        gpio_control_(GPIOControl(pin_, kGPIOOut, kGPIOLow)),
        pwm_timer_(event_loop_->AddTimer([this]() {
          gpio_control_.GPIOWrite(leds_on_ ? kGPIOHigh : kGPIOLow);
          int period_us = static_cast<int>(1000000.0 / frequency_);
          int on_time_us =
              static_cast<int>(duty_cycle_ * 1000000.0 / frequency_);

          // Trigger the next change
          // If the leds are currently off, turn them on for duty_cycle % of
          // period If they are currently on, turn them off for 1 -
          // duty_cycle % of period
          pwm_timer_->Schedule(
              event_loop_->monotonic_now() +
              std::chrono::microseconds(leds_on_ ? (period_us - on_time_us)
                                                 : on_time_us));
          leds_on_ = !leds_on_;
        })) {
    pwm_timer_->Schedule(event_loop_->monotonic_now());
  };

  void set_duty_cycle(double duty_cycle) { duty_cycle_ = duty_cycle; }
  void set_frequency(double frequency) { frequency_ = frequency; }

  aos::ShmEventLoop *const event_loop_;
  int pin_;
  double frequency_;
  double duty_cycle_;
  bool leds_on_;
  GPIOControl gpio_control_;
  aos::TimerHandler *const pwm_timer_;
};

}  // namespace vision
}  // namespace y2022
#endif  // Y2022_VISION_GPIO_H_
