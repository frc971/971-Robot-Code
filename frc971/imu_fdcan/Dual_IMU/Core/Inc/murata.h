/*
 * Constants and helpers for the Murata IMU.
 *
 * Datasheet:
 * https://sensorsandpower.angst-pfister.com/fileadmin/products/datasheets/191/SCHA63T-K03-rev3_1640-21648-0029-E-1121.pdf
 *
 *  Created on: Dec 26, 2023
 *      Author: FRC 971 - Zachary Berthillier & Sindy Tan
 */

#ifndef INC_MURATA_H_
#define INC_MURATA_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "math.h"

#define CONV_INT16_INT8_H(a) ((int8_t)(((a) >> 8) & 0xFF))
#define CONV_INT16_INT8_L(a) ((int8_t)(a & 0xFF))
#define CONV_UINT8_INT16(a_H, a_L) (int16_t)((a_H << 8) | a_L)
#define CONV_UINT16_UINT8_H(a) ((uint8_t)(((a) >> 8) & 0xFF))
#define CONV_UINT16_UINT8_L(a) ((uint8_t)(a & 0xFF))
#define CONV_UINT16_INT8_H(a) ((int8_t)(((a) >> 8) & 0xFF))
#define CONV_UINT16_INT8_L(a) ((int8_t)(a & 0xFF))

#define GET_SPI_DATA_INT16(a) ((int16_t)(((a) >> 8) & 0xFFFF))
#define GET_SPI_DATA_UINT16(a) ((uint16_t)(((a) >> 8) & 0xFFFF))
#define GET_SPI_DATA_UINT8_H(a) ((uint8_t)((a >> 16) & 0xFF))
#define GET_SPI_DATA_UINT8_L(a) ((uint8_t)((a >> 8) & 0xFF))
#define CHECK_RS_ERROR(a) (!(((a) >> 24) & 0x03))
#define MURATA_CONV_TEMP(a) (float)((a / 30.0f) + 25)
#define MURATA_CONV_GYRO(a) (float)(a / 80.0f) * ((float)(M_PI) / 180.0f)
#define MURATA_CONV_ACC(a) (float)(a / 4905.0f)

// Define each operation as SPI 32-bit frame. Details in datasheet page 34
#define WRITE_REG_BANK_0 0xFC000073
#define WRITE_SEL_BANK_5 0x1F000005
#define WRITE_RESET 0xE000017C
#define WRITE_OP_MODE_NORMAL 0xE4000067
#define WRITE_FILTER_13HZ_GYRO 0xD8000045  // Set 13 Hz filter
#define WRITE_FILTER_46HZ_GYRO 0xD812129E  // Set 46 Hz filter
#define WRITE_FILTER_13HZ_ACC 0xE800006D   // Set 13 Hz filter
#define WRITE_FILTER_46HZ_ACC 0xE8022248   // Set 46 Hz filter
#define WRITE_EOI_BIT 0xE000025B

#define MURATA_GYRO_FILTER_ADDR 0x36
#define MURATA_GYRO_FILTER_300HZ 0x2424
#define MURATA_ACC_FILTER_ADDR 0xE8
#define MURATA_ACC_FILTER_300HZ 0x0444

#define READ_GYRO_X 0x040000F7
#define READ_GYRO_Y 0x0C0000FB
#define READ_GYRO_Z 0x040000F7
#define READ_ACC_X 0x100000E9
#define READ_ACC_Y 0x140000EF
#define READ_ACC_Z 0x180000E5
#define READ_TEMP 0x1C0000E3

#define READ_SUMMARY_STATUS 0x380000D5
#define READ_TRC_0 0x740000BF
#define READ_TRC_1 0x780000B5
#define READ_TRC_2 0x700000B9

#define READ_MODE 0x640000A7
#define WRITE_MODE_ASM_010 0xE40010AA
#define WRITE_MODE_ASM_001 0xE400088F
#define WRITE_MODE_ASM_100 0xE40020E0

#define ACC_DC1_ADDRESS 0x0B   // cxx & cxy
#define ACC_DC9_ADDRESS 0x13   // cxz & cyx
#define ACC_DC10_ADDRESS 0x14  // cyy & cyz
#define ACC_DC11_ADDRESS 0x15  // czx & czy
#define ACC_DC12_ADDRESS 0x16  // czz & bxx
#define ACC_DC13_ADDRESS 0x17  // bxy & bxz
#define ACC_DC14_ADDRESS 0x18  // byx & byy
#define ACC_MD1_ADDRESS 0x1B   // byz & bzx
#define ACC_MD2_ADDRESS 0x1C   // bzy & bzz

// Raw sensor data received over SPI
typedef struct {
  int16_t acc_x;     // g
  int16_t acc_y;     // g
  int16_t acc_z;     // g
  int16_t gyro_x;    // deg/s
  int16_t gyro_y;    // deg/s
  int16_t gyro_z;    // deg/s
  int16_t due_temp;  // C
  int16_t uno_temp;  // C
} DataMurataRaw;

// Human readable sensor data
typedef struct {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float due_temp;
  float uno_temp;
} DataMurata;

// Cross axis compensation values for fine tuning acc and gyro output
// See datasheet page 11 for math and details
typedef struct {
  float cxx;
  float cxy;
  float cxz;
  float cyx;
  float cyy;
  float cyz;
  float czx;
  float czy;
  float czz;
  float bxx;
  float bxy;
  float bxz;
  float byx;
  float byy;
  float byz;
  float bzx;
  float bzy;
  float bzz;
} CrossAxisCompMurata;

#endif /* INC_MURATA_H_ */
