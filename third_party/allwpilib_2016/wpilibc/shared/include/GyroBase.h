/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "SensorBase.h"
#include "PIDSource.h"
#include "interfaces/Gyro.h"
#if FULL_WPILIB
#include "LiveWindow/LiveWindowSendable.h"
#endif

#include <memory>

/**
 * GyroBase is the common base class for Gyro implementations such as
 * AnalogGyro.
 */
class GyroBase : public Gyro,
                 public SensorBase,
                 public PIDSource
#if FULL_WPILIB
                 ,
                 public LiveWindowSendable
#endif
                 {
 public:
  virtual ~GyroBase() = default;

  // PIDSource interface
  double PIDGet() override;

#if FULL_WPILIB
  void UpdateTable() override;
  void StartLiveWindowMode() override;
  void StopLiveWindowMode() override;
  std::string GetSmartDashboardType() const override;
  void InitTable(std::shared_ptr<ITable> subTable) override;
  std::shared_ptr<ITable> GetTable() const override;

 private:
  std::shared_ptr<ITable> m_table;
#endif
};
