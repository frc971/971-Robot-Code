/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Global.h"
#include "Utility.h"

Global *Global::instance;
ReentrantSemaphore Global::instance_lock;

Global *Global::GetInstance() {
  Synchronized sync(instance_lock);
  if (instance == NULL) {
    instance = new Global();
  }
  return instance;
}

Global::Global() {
	tRioStatusCode status = NiFpga_Status_Success;
	global_.reset(tGlobal::create(&status));
  wpi_setError(status);

  AddToSingletonList();
}

Global::~Global() {
  Synchronized sync(instance_lock);
  instance = NULL;
}

/**
 * Return the FPGA Version number.
 * For now, expect this to be competition year.
 * @return FPGA Version number.
 */
UINT16 Global::GetFPGAVersion()
{
	tRioStatusCode status = NiFpga_Status_Success;
	UINT16 version = global_->readVersion(&status);
	wpi_setError(status);
	return version;
}

/**
 * Return the FPGA Revision number.
 * The format of the revision is 3 numbers.
 * The 12 most significant bits are the Major Revision.
 * the next 8 bits are the Minor Revision.
 * The 12 least significant bits are the Build Number.
 * @return FPGA Revision number.
 */
UINT32 Global::GetFPGARevision()
{
	tRioStatusCode status = NiFpga_Status_Success;
	UINT32 revision = global_->readRevision(&status);
	wpi_setError(status);
	return revision;
}

/**
 * Read the microsecond-resolution timer on the FPGA.
 *
 * @return The current time in microseconds according to the FPGA (since FPGA reset).
 */
UINT32 Global::GetFPGATime()
{
	tRioStatusCode status = NiFpga_Status_Success;
	UINT32 time = global_->readLocalTime(&status);
	wpi_setError(status);
	return time;
}

// RT hardware access functions exported from ni_emb.out
extern "C"
{
	INT32 UserSwitchInput(INT32 nSwitch);
	INT32 LedInput(INT32 led);
	INT32 LedOutput(INT32 led, INT32 value);
}

/**
 * Read the value of the USER1 DIP switch on the cRIO.
 */
INT32 Global::GetRIOUserSwitch()
{
	INT32 switchValue = UserSwitchInput(0);
	wpi_assert(switchValue >= 0);
	return switchValue > 0;
}

/**
 * Set the state of the USER1 status LED on the cRIO.
 */
void Global::SetRIOUserLED(UINT32 state)
{
	LedOutput(0, state > 0);
}

/**
 * Get the current state of the USER1 status LED on the cRIO.
 * @return The curent state of the USER1 LED.
 */
INT32 Global::GetRIOUserLED()
{
	return LedInput(0);
}

/**
 * Toggle the state of the USER1 status LED on the cRIO.
 * @return The new state of the USER1 LED.
 */
INT32 Global::ToggleRIOUserLED()
{
  Synchronized sync(led_toggle_lock_);
	INT32 ledState = !GetRIOUserLED();
	SetRIOUserLED(ledState);
	return ledState;
}

/**
 * Set the state of the FPGA status LED on the cRIO.
 */
void Global::SetRIO_FPGA_LED(UINT32 state)
{
	tRioStatusCode status = NiFpga_Status_Success;
	global_->writeFPGA_LED(state, &status);
	wpi_setError(status);
}

/**
 * Get the current state of the FPGA status LED on the cRIO.
 * @return The curent state of the FPGA LED.
 */
INT32 Global::GetRIO_FPGA_LED()
{
	tRioStatusCode status = NiFpga_Status_Success;
	bool ledValue = global_->readFPGA_LED(&status);
	wpi_setError(status);
	return ledValue;
}

/**
 * Toggle the state of the FPGA status LED on the cRIO.
 * @return The new state of the FPGA LED.
 */
INT32 Global::ToggleRIO_FPGA_LED()
{
  Synchronized sync(led_toggle_lock_);
	INT32 ledState = !GetRIO_FPGA_LED();
	SetRIO_FPGA_LED(ledState);
	return ledState;
}
