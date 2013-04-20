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

  UINT16 GetFPGAVersion();
  UINT32 GetFPGARevision();
  UINT32 GetFPGATime();
  INT32 GetRIOUserSwitch();
  void SetRIOUserLED(UINT32 state);
  INT32 GetRIOUserLED();
  INT32 ToggleRIOUserLED();
  void SetRIO_FPGA_LED(UINT32 state);
  INT32 GetRIO_FPGA_LED();
  INT32 ToggleRIO_FPGA_LED();

 private:
  Global();
  ~Global();

  static Global *instance;
  static ReentrantSemaphore instance_lock;

  ::std::auto_ptr<tGlobal> global_;
  ReentrantSemaphore led_toggle_lock_;
};

#endif  // WPILIB_GLOBAL_H_
