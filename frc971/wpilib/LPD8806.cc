#include "frc971/wpilib/LPD8806.h"

#include "frc971/queues/gyro.q.h"

#include "SPI.h"
#undef ERROR

namespace frc971 {
namespace wpilib {

LPD8806::LPD8806(int chips)
    : chips_(chips),
      data_(new LED[chips * 2]),
      spi_(new SPI(SPI::kOnboardCS1)) {
  memset(data_.get(), 0, sizeof(LED[chips_ * 2]));

  // 2 MHz is the maximum frequency the datasheet recommends.
  spi_->SetClockRate(2e6);
  spi_->SetChipSelectActiveHigh();

  // Clock is inverted due to the level translator chip.
  spi_->SetClockActiveLow();
  spi_->SetSampleDataOnRising();
  spi_->SetMSBFirst();
}

void LPD8806::SetColor(int led, uint32_t hex_color) {
  CHECK_LT(led, chips_ * 2);
  ::aos::MutexLocker locker(&data_mutex_);
  data_[led].red = TranslateColor(hex_color, RED);
  data_[led].green = TranslateColor(hex_color, GREEN);
  data_[led].blue = TranslateColor(hex_color, BLUE);
}

uint8_t LPD8806::TranslateColor(uint32_t hex_color, Type type) {
  switch (type) {
    case RED:
      return ((hex_color >> 16) & 0xFF) | 0x01;
    case GREEN:
      return ((hex_color >> 8) & 0xFF) | 0x01;
    case BLUE:
      return (hex_color & 0xFF) | 0x01;
  }

  LOG(FATAL, "Not sure what color type %d is\n", static_cast<int>(type));
}

void LPD8806::operator()() {
  // The buffer we're going to send.
  // With 64 leds, it takes about 1ms to send them all, which fits within the
  // 5ms cycle used by the gyro_reader.
  LED buffer[64];

  while (run_) {
    if (next_led_to_send_ < chips_ * 2) {
      ::aos::MutexLocker locker(&data_mutex_);
      memcpy(buffer, &data_[next_led_to_send_], sizeof(buffer));
      next_led_to_send_ += 64;
    } else {
      CHECK_EQ(chips_ * 2, next_led_to_send_);
      next_led_to_send_ = 0;
      memset(buffer, 0, sizeof(buffer));
    }

    // Wait until right after the gyro gets a reading.
    ::frc971::sensors::gyro_reading.FetchNextBlocking();

    int spi_send_result = (spi_->Write(
        static_cast<uint8_t *>(static_cast<void *>(buffer)), sizeof(buffer)));

    switch (spi_send_result) {
      case -1:
        LOG(INFO, "SPI::Write failed\n");
        // Who knows what state it's in now, so start fresh next cycle.
        next_led_to_send_ = chips_ * 2;
        break;
      case sizeof(buffer):
        break;
      default:
        LOG(FATAL, "SPI::Write returned something weird: %d\n",
            static_cast<int>(spi_send_result));
    }
  }
}

}  // namespace wpilib
}  // namespace frc971
