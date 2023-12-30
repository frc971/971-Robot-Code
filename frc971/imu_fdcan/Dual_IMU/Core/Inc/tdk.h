/*
 * Constants and helpers for the TDK IMU.
 *
 * Datasheet:
 * https://invensense.tdk.com/wp-content/uploads/2021/11/DS-000409-IAM-20680HP-v1.2-Typ.pdf
 *
 *  Created on: Dec 26, 2023
 *      Author: FRC 971 - Zachary Berthillier & Sindy Tan
 */

#ifndef INC_TDK_H_
#define INC_TDK_H_

#define CONV_UINT8_INT16(a_H, a_L) (int16_t)((a_H << 8) | a_L)

// Convert raw sensor data to human readable format
#define TDK_CONV_TEMP(a) (float)((a / 326.8f) + 25)  // output in Celsius
#define TDK_CONV_GYRO(a) (float)(a / 16.4f)          // output in deg/s
#define TDK_CONV_ACC(a) (float)(a / 2048.0f)         // output in g

// Register map
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG_2 0x1D
#define FIFO_EN 0x23
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define FIFO_COUNT_H 0x72
#define FIFO_COUNT_L 0x73
#define FIFO_R_W 0x74

// Raw sensor data received over SPI
typedef struct {
  uint8_t acc_x_L;
  uint8_t acc_x_H;
  uint8_t acc_y_L;
  uint8_t acc_y_H;
  uint8_t acc_z_L;
  uint8_t acc_z_H;
  uint8_t gyro_x_L;
  uint8_t gyro_x_H;
  uint8_t gyro_y_L;
  uint8_t gyro_y_H;
  uint8_t gyro_z_L;
  uint8_t gyro_z_H;
  uint8_t temp_L;
  uint8_t temp_H;
} DataTdkRaw;

// Human readable sensor data
typedef struct {
  float acc_x;   // g
  float acc_y;   // g
  float acc_z;   // g
  float gyro_x;  // deg/s
  float gyro_y;  // deg/s
  float gyro_z;  // deg/s
  float temp;    // C
} DataTdk;

#endif /* INC_TDK_H_ */
