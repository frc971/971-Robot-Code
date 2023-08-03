/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include <algorithm>
#include <cstring>

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "pico/bootrom.h"
#include "pico/double.h"
#include "pico/stdlib.h"
#include "quadrature_encoder.pio.h"

#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic ignored "-Wformat-extra-args"

// Pinout definitions for the imu
#define RST_IMU 22
#define DR_IMU 20
#define SYNC_IMU 21
#define DIN_IMU 19
#define DOUT_IMU 16
#define SCLK_IMU 18
#define CS_IMU 17

// Pinout definitions for spi to the pi through the differential drivers
#define DR_PI 14
#define MOSI_PI 12
#define MISO_PI 11
#define SCK_PI 10
#define CS_PI 13

// The two drivetrain encoders
#define ENC1_A 6
#define ENC1_B 7
#define ENC2_A 0
#define ENC2_B 1

// Backup outputs to the roborio
#define RATE_PWM 2
#define HEADING_PWM 4

#define PWM_FREQ_HZ 200
// PWM counts to this before wrapping
#define PWM_TOP 62499

#define SPI_IMU spi0
#define SPI_PI spi1
#define WRITE_BIT 0x8000

// length in half-words of the buffer for communicating with the IMU
// includes 2 non-data fields
// the first element is used for recieving zeros while the initial request is
// made the last element is the checksum
#define IMU_NUM_ITEMS 17

// length in bytes of the packet to the pi
#define PI_NUM_ITEMS 42

// number of samples for zeroing the z-gyro axis
#define YAW_BUF_LEN 5000

#define EXPECTED_PROD_ID 0x4079

// Registers on the ADIS16505
#define GLOB_CMD 0x68
#define DIAG_STAT 0x02
#define FIRM_REV 0x6C
#define FIRM_DM 0x6E
#define FIRM_Y 0x70
#define PROD_ID 0x72
#define SERIAL_NUM 0x74
#define FILT_CTRL 0x5C
#define MSC_CTRL 0x60
#define DEC_RATE 0x64

// TODO: replace with aos/events/logging/crc32.h
const uint32_t kCrc32Table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

// Buffer to start a burst read and recive 15 items + checksum
static uint16_t imu_write_buf[IMU_NUM_ITEMS] = {0x6800, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0,      0, 0, 0, 0, 0, 0, 0};
// recieves a byte of zeros followed by 15 items + checksum
static uint16_t imu_data_buffer[IMU_NUM_ITEMS];

static dma_channel_config imu_tx_config;
static dma_channel_config imu_rx_config;
static uint imu_dma_tx;
static uint imu_dma_rx;

// The packet to the pi contains the whole burst read from the IMU (30 bytes)
// followed by a timestamp, encoder values, and checksum
// DIAG_STAT, X_GYRO_LOW, X_GYRO_OUT, Y_GYRO_LOW, Y_GYRO_OUT, Z_GYRO_LOW,
// Z_GYRO_OUT, X_ACCL_LOW, X_ACCL_OUT, Y_ACCL_LOW, Y_ACCL_OUT, Z_ACCL_LOW,
// Z_ACCL_OUT, TEMP_OUT, DATA_CNTR, TIMESTAMP (32 bit), ENC1_POS, ENC2_POS,
// CHECKSUM (32-bit)

// the staging buffer gets updated everytime new data is received
static uint8_t pi_staging_buffer[PI_NUM_ITEMS];
// the data from the staging buffer is latched into the sending buffer while the
// pi is reading
static uint8_t pi_sending_buffer[PI_NUM_ITEMS];

// for now just recieves zeros
// but is useful because directing the dma to a nullptr messes up the transfer
// finished interrupt
static uint8_t pi_recieve_buffer[PI_NUM_ITEMS];

static dma_channel_config pi_tx_config;
static dma_channel_config pi_rx_config;
static uint pi_dma_tx;
static uint pi_dma_rx;

// yaw velocity from the latest message from the imu
static double yaw_rate = 0;

static int16_t encoder1_count = 0;
static int16_t encoder2_count = 0;

static uint32_t data_collect_timestamp = 0;

// if we need to reset the imu
static bool reset_imu = false;
// number of consectuive checksums that are mismatched or zero
static uint suspicious_checksums = 0;

// useful debug counters
static uint imu_reset_count = 0;
static uint checksum_mismatch_count = 0;
static uint message_recieved_count = 0;
static uint message_sent_count = 0;
// the number of times we had to defer sending new data because another
// transfer was still in progress
static uint timing_overrun_count = 0;

// the time it takes from servicing DR_IMU to finishing the transfer to the pi
static int send_time = 0;

void data_ready();
void maybe_send_pi_packet();

void gpio_irq_handler(uint gpio, uint32_t events) {
  if (gpio == SYNC_IMU && (events & GPIO_IRQ_EDGE_RISE)) {
    // Grab a timestamp for when the data sample was actually collected.
    data_collect_timestamp = time_us_32();
  }
  if (gpio == DR_IMU && (events & GPIO_IRQ_EDGE_RISE)) {
    data_ready();
  }
}

static inline void cs_select() {
  gpio_put(CS_IMU, 0);  // Active low
  sleep_us(1);
}

static inline void cs_deselect() { gpio_put(CS_IMU, 1); }

static uint16_t read_register(uint8_t reg_low) {
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device.
  uint16_t output;

  // The low byte of the register is first in the IMU's memory
  uint16_t reg = ((uint16_t)reg_low) << 8;

  cs_select();
  spi_write16_blocking(SPI_IMU, &reg, 1);
  cs_deselect();
  sleep_us(20);  // wait the stall period
  cs_select();
  spi_read16_blocking(SPI_IMU, 0x0000, &output, 1);
  cs_deselect();

  return output;
}

static void write_register(uint8_t reg, uint16_t value) {
  uint16_t low_byte = ((uint16_t)value) & 0xFF;
  uint16_t high_byte = ((uint16_t)value) >> 8;

  uint16_t command = (((uint16_t)reg) << 8) | low_byte | WRITE_BIT;

  cs_select();
  spi_write16_blocking(SPI_IMU, &command, 1);
  cs_deselect();

  sleep_us(20);  // wait the stall period

  command = ((((uint16_t)reg) + 1) << 8) | high_byte | WRITE_BIT;

  cs_select();
  spi_write16_blocking(SPI_IMU, &command, 1);
  cs_deselect();
}

static void adis16505_reset() {
  gpio_put(RST_IMU, 0);  // Active low
  sleep_ms(10);
  gpio_put(RST_IMU, 1);  // Active low
  sleep_ms(310);
}

static uint32_t calculate_crc32(uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc = kCrc32Table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc;
}

static uint16_t check_checksum(uint16_t *buf) {
  uint16_t sum = 0;
  for (int i = 1; i < IMU_NUM_ITEMS - 1; i++) {
    uint16_t low = buf[i] & 0xff;
    uint16_t high = (buf[i] >> 8) & 0xff;

    sum += high + low;
  }

  return sum;
}

void pack_pi_packet() {
  // zero the buffer
  for (int i = 0; i < PI_NUM_ITEMS; i++) {
    pi_staging_buffer[i] = 0;
  }

  // skip empty item
  uint8_t *imu_packet = (uint8_t *)(imu_data_buffer + 1);
  // skip empty item and checksum and convert to bytes
  const size_t imu_packet_len = (IMU_NUM_ITEMS - 2) * sizeof(uint16_t);

  memcpy(pi_staging_buffer, imu_packet, imu_packet_len);
  memcpy(pi_staging_buffer + imu_packet_len, &data_collect_timestamp, 4);
  memcpy(pi_staging_buffer + imu_packet_len + 4, &encoder1_count, 2);
  memcpy(pi_staging_buffer + imu_packet_len + 6, &encoder2_count, 2);

  // exclude the part of the buffer that will be the checksum
  uint32_t crc = calculate_crc32(pi_staging_buffer, PI_NUM_ITEMS - 4);
  memcpy(pi_staging_buffer + imu_packet_len + 8, &crc, 4);

  static_assert(PI_NUM_ITEMS == imu_packet_len +
                                    sizeof(data_collect_timestamp) +
                                    sizeof(encoder1_count) +
                                    sizeof(encoder2_count) + sizeof(crc),
                "PI_NUM_ITEMS needs to be able to hold the imu message + the "
                "timestamp + the encoders + the checksum");
}

void data_ready() {
  // read encoders
  quadrature_encoder_request_count(pio0, 0);
  quadrature_encoder_request_count(pio0, 1);

  cs_select();
  dma_channel_configure(imu_dma_tx, &imu_tx_config,
                        &spi_get_hw(SPI_IMU)->dr,  // write address
                        &imu_write_buf,            // read address
                        IMU_NUM_ITEMS,  // element count (each element is of
                                        // size transfer_data_size)
                        false);         // don't start yet
  dma_channel_configure(imu_dma_rx, &imu_rx_config,
                        &imu_data_buffer,          // write address
                        &spi_get_hw(SPI_IMU)->dr,  // read address
                        IMU_NUM_ITEMS,  // element count (each element is of
                                        // size transfer_data_size)
                        false);         // don't start yet
  dma_start_channel_mask((1u << imu_dma_tx) | (1u << imu_dma_rx));

  encoder1_count = quadrature_encoder_fetch_count(pio0, 0);
  encoder2_count = quadrature_encoder_fetch_count(pio0, 1);
}

void imu_read_finished() {
  cs_deselect();

  // TODO: check status and if necessary set flag to reset in main loop

  message_recieved_count++;
  uint16_t computed_checksum = check_checksum(imu_data_buffer);
  uint16_t given_checksum = imu_data_buffer[IMU_NUM_ITEMS - 1];

  if (computed_checksum != given_checksum) {
    checksum_mismatch_count++;
    for (size_t i = 0; i < IMU_NUM_ITEMS; i++) {
      // make it clear that this data is bad
      imu_data_buffer[i] = 0;
    }
    // and set an unused bit of DIAG_STAT
    imu_data_buffer[1] = 1u << 0;
  } else {
    int32_t z_gyro_out;
    memcpy(&z_gyro_out, imu_data_buffer + 6, 4);
    yaw_rate = (double)z_gyro_out / 655360.0;  // degrees

    // 50% is 0; -2000 deg/sec to 2000 deg/sec
    double scaled_rate = (std::clamp(yaw_rate, -2000.0, 2000.0) / 4000.0 + 0.5);

    constexpr double kScaledRangeLow = 0.1;
    constexpr double kScaledRangeHigh = 0.9;

    uint16_t rate_level =
        (scaled_rate * (kScaledRangeHigh - kScaledRangeLow) + kScaledRangeLow) *
        PWM_TOP;
    pwm_set_gpio_level(RATE_PWM, rate_level);
  }

  // if 5 or more consecutive checksums are zero, then something weird is going
  // on with the imu
  if (computed_checksum != given_checksum || computed_checksum == 0 ||
      given_checksum == 0) {
    suspicious_checksums++;
  } else {
    suspicious_checksums = 0;
  }

  if (suspicious_checksums >= 5) {
    reset_imu = true;
  }

  // fill out message and add it to the queue
  pack_pi_packet();

  // TODO: this has a sleep in it
  // stay in sync with the pi by resetting the spi fifos each transfer
  spi_init(SPI_PI, 2000 * 1000);
  spi_set_slave(SPI_PI, true);
  // these settings were the most stable even though the PI is using
  // polarity 1 and phase 1
  spi_set_format(SPI_PI, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

  maybe_send_pi_packet();

  // clear the interrupt
  dma_hw->ints0 = 1u << imu_dma_rx;
}

void maybe_send_pi_packet() {
  // active low; if the pi isn't connected/booted, this will
  // also look active
  bool cs_asserted_or_unplugged = !gpio_get(CS_PI);

  if (cs_asserted_or_unplugged) {
    // the pi is still recieving something else from us
    timing_overrun_count++;
    return;
  }

  memcpy(pi_sending_buffer, pi_staging_buffer, PI_NUM_ITEMS);

  dma_channel_configure(pi_dma_tx, &pi_tx_config,
                        &spi_get_hw(SPI_PI)->dr,  // write address
                        &pi_sending_buffer,       // read address
                        PI_NUM_ITEMS,  // element count (each element is
                                       // of size transfer_data_size)
                        false);        // don't start yet
  dma_channel_configure(pi_dma_rx, &pi_rx_config,
                        &pi_recieve_buffer,       // write address
                        &spi_get_hw(SPI_PI)->dr,  // read address
                        PI_NUM_ITEMS,  // element count (each element is of
                                       // size transfer_data_size)
                        false);        // don't start yet

  // start hardware calculation of the CRC-32; currently calculated in software
  dma_sniffer_enable(pi_dma_tx, 0x0, true);
  dma_hw->sniff_data = 0;

  // start both at exactly the same time
  dma_start_channel_mask((1u << pi_dma_tx) | (1u << pi_dma_rx));
  gpio_put(DR_PI, 1);
}

void pi_transfer_finished() {
  message_sent_count++;
  gpio_put(DR_PI, 0);

  send_time = time_us_32() - data_collect_timestamp;

  dma_hw->ints1 = 1u << pi_dma_rx;
}

void setup_adis16505() {
  // Disable the interrupts from the data-ready/sync pins to avoid interrupts
  // while attempting to reset the IMU.
  gpio_set_irq_enabled(DR_IMU, GPIO_IRQ_EDGE_RISE, false);
  gpio_set_irq_enabled(SYNC_IMU, GPIO_IRQ_EDGE_RISE, false);

  while (true) {
    adis16505_reset();
    //   See if SPI is working - interrogate the device for its product ID
    //   number, should be 0x4079
    uint16_t id = read_register(PROD_ID);
    if (id == EXPECTED_PROD_ID) {
      printf("Product id: 0x%04x == expected 0x%04x\n", id, EXPECTED_PROD_ID);
      break;
    } else {
      printf("Got 0x%04x for prod id, expected 0x%04x\ntrying again\n", id,
             EXPECTED_PROD_ID);
    }
  }

  uint16_t firmware_revision = read_register(FIRM_REV);
  uint16_t firmware_day_month = read_register(FIRM_DM);
  uint16_t firmware_year = read_register(FIRM_Y);
  uint16_t serial_number = read_register(SERIAL_NUM);

  printf(
      "Firmware revision: 0x%04x, \nFirmware day month: 0x%04x, \nFirmware "
      "year: "
      "0x%04x, \nSerial number: 0x%04x, \n",
      firmware_revision, firmware_day_month, firmware_year, serial_number);

  // run self test
  int num_failures = 0;
  while (true) {
    write_register(GLOB_CMD, 1u << 2);
    sleep_ms(24);
    uint16_t diag_stat = read_register(DIAG_STAT);

    // check the sensor failure bit
    bool sensor_failure = diag_stat & (1u << 5);
    printf("Diag stat: 0b%016b, \n", diag_stat);

    if (sensor_failure) {
      num_failures++;
      printf("%d failures, trying again\n", num_failures);
    } else {
      break;
    }
  }

  write_register(FILT_CTRL, 0 /* no filtering */);
  write_register(
      MSC_CTRL,
      (1u << 9) /* enable 32-bit mode for burst reads */ |
          (0u << 8) /* send gyro and accelerometer data in burst mode */ |
          (1u << 7) /* enable gyro linear g compensation */ |
          (1u << 6) /* enable point of percussion alignment */ |
          (11u << 2) /* output sync mode (uses internal 2kHz clock) */ |
          (1u << 1) /* sync polarity, active high */ |
          (1u << 0) /* data ready is active high */);
  // Rate of the output will be 2000 / (DEC_RATE + 1) Hz.
  write_register(DEC_RATE, 0 /* no decimation */);

  sleep_us(200);

  imu_reset_count++;

  gpio_set_irq_enabled(SYNC_IMU, GPIO_IRQ_EDGE_RISE, true);
  gpio_set_irq_enabled_with_callback(DR_IMU, GPIO_IRQ_EDGE_RISE, true,
                                     &gpio_irq_handler);
}

int main() {
  stdio_init_all();

  // Use 1MHz with the IMU as that is the limit for burst mode.
  spi_init(SPI_IMU, 1000 * 1000);
  spi_set_format(SPI_IMU, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(DOUT_IMU, GPIO_FUNC_SPI);
  gpio_set_function(SCLK_IMU, GPIO_FUNC_SPI);
  gpio_set_function(DIN_IMU, GPIO_FUNC_SPI);
  // Make the SPI pins available to picotool
  bi_decl(bi_3pins_with_func(DOUT_IMU, DIN_IMU, SCLK_IMU, GPIO_FUNC_SPI));

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(CS_IMU);
  gpio_set_dir(CS_IMU, GPIO_OUT);
  gpio_put(CS_IMU, 1);
  // Make the CS pin available to picotool
  bi_decl(bi_1pin_with_name(CS_IMU, "IMU CS"));

  // Reset is active-low, so we'll initialise it to a driven-high state
  gpio_init(RST_IMU);
  gpio_set_dir(RST_IMU, GPIO_OUT);
  gpio_put(RST_IMU, 1);
  // Make the RST pin available to picotool
  bi_decl(bi_1pin_with_name(RST_IMU, "IMU RESET"));

  imu_dma_tx = dma_claim_unused_channel(true);
  imu_dma_rx = dma_claim_unused_channel(true);

  // We set the outbound DMA to transfer from a memory buffer to the SPI
  // transmit FIFO paced by the SPI TX FIFO DREQ The default is for the read
  // address to increment every element (in this case 2 bytes - DMA_SIZE_16) and
  // for the write address to remain unchanged.

  imu_tx_config = dma_channel_get_default_config(imu_dma_tx);
  channel_config_set_transfer_data_size(&imu_tx_config, DMA_SIZE_16);
  channel_config_set_dreq(&imu_tx_config, spi_get_dreq(SPI_IMU, true));
  channel_config_set_read_increment(&imu_tx_config, true);
  channel_config_set_write_increment(&imu_tx_config, false);

  // We set the inbound DMA to transfer from the SPI receive FIFO to a memory
  // buffer paced by the SPI RX FIFO DREQ We configure the read address to
  // remain unchanged for each element, but the write address to increment (so
  // data is written throughout the buffer)
  imu_rx_config = dma_channel_get_default_config(imu_dma_rx);
  channel_config_set_transfer_data_size(&imu_rx_config, DMA_SIZE_16);
  channel_config_set_dreq(&imu_rx_config, spi_get_dreq(SPI_IMU, false));
  channel_config_set_read_increment(&imu_rx_config, false);
  channel_config_set_write_increment(&imu_rx_config, true);

  // Tell the DMA to raise IRQ line 0 when the channel finishes a block
  dma_channel_set_irq0_enabled(imu_dma_rx, true);

  // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
  irq_set_exclusive_handler(DMA_IRQ_0, imu_read_finished);
  irq_set_enabled(DMA_IRQ_0, true);

  // Use 2MHz with the PI as that is the maximum speed for the pico
  gpio_set_function(MISO_PI, GPIO_FUNC_SPI);
  gpio_set_function(MOSI_PI, GPIO_FUNC_SPI);
  gpio_set_function(SCK_PI, GPIO_FUNC_SPI);
  gpio_set_function(CS_PI, GPIO_FUNC_SPI);
  // Make the SPI pins available to picotool
  bi_decl(bi_3pins_with_func(DOUT_IMU, DIN_IMU, SCLK_IMU, GPIO_FUNC_SPI));

  gpio_init(DR_PI);
  gpio_set_dir(DR_PI, GPIO_OUT);
  gpio_put(DR_PI, 0);
  // Make the CS pin available to picotool
  bi_decl(bi_1pin_with_name(DR_PI, "DATA READY PI"));

  pi_dma_tx = dma_claim_unused_channel(true);
  pi_dma_rx = dma_claim_unused_channel(true);

  // We set the outbound DMA to transfer from a memory buffer to the SPI
  // transmit FIFO paced by the SPI TX FIFO DREQ The default is for the read
  // address to increment every element (in this case 2 bytes - DMA_SIZE_16) and
  // for the write address to remain unchanged.

  pi_tx_config = dma_channel_get_default_config(pi_dma_tx);
  channel_config_set_transfer_data_size(&pi_tx_config, DMA_SIZE_8);
  channel_config_set_dreq(&pi_tx_config, spi_get_dreq(SPI_PI, true));
  channel_config_set_read_increment(&pi_tx_config, true);
  channel_config_set_write_increment(&pi_tx_config, false);

  // We set the inbound DMA to transfer from the SPI receive FIFO to a memory
  // buffer paced by the SPI RX FIFO DREQ We configure the read address to
  // remain unchanged for each element, but the write address to increment (so
  // data is written throughout the buffer)
  pi_rx_config = dma_channel_get_default_config(pi_dma_rx);
  channel_config_set_transfer_data_size(&pi_rx_config, DMA_SIZE_8);
  channel_config_set_dreq(&pi_rx_config, spi_get_dreq(SPI_PI, false));
  channel_config_set_read_increment(&pi_rx_config, false);
  channel_config_set_write_increment(&pi_rx_config, true);

  // Tell the DMA to raise IRQ line 1 when the channel finishes a block
  dma_channel_set_irq1_enabled(pi_dma_rx, true);

  // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
  irq_set_exclusive_handler(DMA_IRQ_1, pi_transfer_finished);
  irq_set_enabled(DMA_IRQ_1, true);

  /* All IRQ priorities are initialized to PICO_DEFAULT_IRQ_PRIORITY by the pico
   * runtime at startup. As such, their priorities will correspond to their
   * IRQ numbers (See Table 80 in the rp2040 datasheet). The interrupts are
   * listed highest priority to lowest below--i.e., the sync/data ready
   * interrupts are currently at the lowest priority.
   * TODO(james): In the nominal case, the GPIO interrupts should never
   * interfere with the SPI data transfer, but we may still want to up the
   * GPIO priority using irq_set_priority() so that we are guaranteed good
   * timestamps.
   *
   * Handler             | Interrupt    | Cause of interrupt
   * --------------------|--------------|---------------------------------------
   * imu_read_finished   | DMA_IRQ_0    | When the dma read from the imu is done
   * pi_transfer_finished| DMA_IRQ_1    | When the dma read to the pi is
   * done data_ready     | IO_IRQ_BANK0 | On the rising edge of DR_IMU
   * sync pin high       | IO_IRQ_BANK0 | On the rising edge of SYNC_IMU
   */

  // Tell the GPIOs they are allocated to PWM
  gpio_set_function(HEADING_PWM, GPIO_FUNC_PWM);
  gpio_set_function(RATE_PWM, GPIO_FUNC_PWM);

  // Find out which PWM slice is connected to each gpio pin
  uint heading_slice = pwm_gpio_to_slice_num(HEADING_PWM);
  uint rate_slice = pwm_gpio_to_slice_num(RATE_PWM);

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

  float divisor = clock_get_hz(clk_sys) / (PWM_TOP + 1) / PWM_FREQ_HZ;

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
  pwm_config_set_clkdiv(&cfg, divisor);
  pwm_config_set_wrap(&cfg, PWM_TOP);

  pwm_init(heading_slice, &cfg, true);
  pwm_init(rate_slice, &cfg, true);

  // the two state machine instances use the same program memory from pio0
  uint offset = pio_add_program(pio0, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio0, 0, offset, ENC1_A, 0);
  quadrature_encoder_program_init(pio0, 1, offset, ENC2_A, 0);

  sleep_ms(3 * 1000);

  printf("Hello ADIS16505\n");

  printf(
      "System clock: %d hz\n"
      "PWM target frequency: %d hz, PWM clock divisor: "
      "%f, PWM TOP: %d\n",
      clock_get_hz(clk_sys), PWM_FREQ_HZ, divisor, PWM_TOP);

  setup_adis16505();

  printf("Press q to enter bootloader\n");

  while (1) {
    dma_channel_wait_for_finish_blocking(imu_dma_rx);
    // we want the interrupts to happen before or during the sleep so that we
    // can get a good picture of what's going on without things moving around
    sleep_us(100);

    if (reset_imu) {
      printf("Triggered IMU reset, resetting\n");
      setup_adis16505();
      reset_imu = false;
    }

    // debug
    // one printf is faster than many printfs
    printf(
        "z vel: %f, encoder: %d %d\n"
        "Num failed checksums: %d, Total messages recieved: %d,\n"
        "Num messages to pi: %d, Timing overrun count: %d,\n"
        "Send time: %d us, suspicious checksum count: %u,\n"
        "IMU reset count: %d, checksum: %u,\n",
        yaw_rate, encoder1_count, encoder2_count, checksum_mismatch_count,
        message_recieved_count, message_sent_count, timing_overrun_count,
        send_time, suspicious_checksums, imu_reset_count,
        imu_data_buffer[IMU_NUM_ITEMS - 1]);

    // allow the user to enter the bootloader without removing power or having
    // to install a reset button
    char user_input = getchar_timeout_us(0);
    if (user_input == 'q') {
      printf("Going down! entering bootloader\n");
      reset_usb_boot(0, 0);
    }

    sleep_ms(50);
  }

  printf("All good\n");
  dma_channel_unclaim(imu_dma_rx);
  dma_channel_unclaim(imu_dma_tx);
  dma_channel_unclaim(pi_dma_rx);
  dma_channel_unclaim(pi_dma_tx);
  return 0;
}
