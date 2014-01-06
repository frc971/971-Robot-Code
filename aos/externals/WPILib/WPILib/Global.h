/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include <memory>

#include "SensorBase.h"
#include "Synchronized.h"
#include "ChipObject.h"

#ifndef WPILIB_GLOBAL_H_
#define WPILIB_GLOBAL_H_

class Global : public SensorBase {
 public:
  static Global *GetInstance();

  uint16_t GetFPGAVersion();
  uint32_t GetFPGARevision();
  uint32_t GetFPGATime();
  int32_t GetRIOUserSwitch();
  void SetRIOUserLED(uint32_t state);
  int32_t GetRIOUserLED();
  int32_t ToggleRIOUserLED();
  void SetRIO_FPGA_LED(uint32_t state);
  int32_t GetRIO_FPGA_LED();
  int32_t ToggleRIO_FPGA_LED();

 private:
  Global();
  ~Global();

  static Global *instance;
  static ReentrantSemaphore instance_lock;

  ::std::auto_ptr<tGlobal> global_;
  ReentrantSemaphore led_toggle_lock_;
};

#endif  // WPILIB_GLOBAL_H_
