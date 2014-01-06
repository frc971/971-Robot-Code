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
	explicit SolenoidBase(uint8_t moduleNumber);
	virtual ~SolenoidBase();

	void Set(uint8_t value, uint8_t mask);

	uint8_t GetAll();
  /**
   * Set the value of all of the solenoids at the same time.
   *
   * @param value The values you want to set all of the solenoids to.
   */
  void SetAll(uint8_t value) { Set(value, 0xFF); }

protected:
	uint32_t m_moduleNumber; ///< Slot number where the module is plugged into the chassis.
	static Resource *m_allocated;

private:
	static tSolenoid *m_fpgaSolenoidModule; ///< FPGA Solenoid Module object.
	static uint32_t m_refCount; ///< Reference count for the chip object.
	static ReentrantSemaphore m_semaphore;
};

#endif
