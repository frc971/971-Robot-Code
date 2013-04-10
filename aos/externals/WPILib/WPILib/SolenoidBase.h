/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef SOLENOID_BASE_H_
#define SOLENOID_BASE_H_

#include "Resource.h"
#include "SensorBase.h"
#include "ChipObject.h"
#include "Synchronized.h"

/**
 * SolenoidBase class is the common base class for the Solenoid and
 * DoubleSolenoid classes.
 * It also supports getting and setting the values of all solenoids on a given
 * module at the same time directly.
 */
class SolenoidBase : public SensorBase {
public:
	explicit SolenoidBase(UINT8 moduleNumber);
	virtual ~SolenoidBase();

	void Set(UINT8 value, UINT8 mask);

	UINT8 GetAll();
  /**
   * Set the value of all of the solenoids at the same time.
   *
   * @param value The values you want to set all of the solenoids to.
   */
  void SetAll(UINT8 value) { Set(value, 0xFF); }

protected:
	UINT32 m_moduleNumber; ///< Slot number where the module is plugged into the chassis.
	static Resource *m_allocated;

private:
	static tSolenoid *m_fpgaSolenoidModule; ///< FPGA Solenoid Module object.
	static UINT32 m_refCount; ///< Reference count for the chip object.
	static ReentrantSemaphore m_semaphore;
};

#endif
