/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2014. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef Compressor_H_
#define Compressor_H_

#include "HAL/HAL.hpp"
#include "SensorBase.h"

/**
 * PCM compressor
 */
class Compressor: public SensorBase {
public:
	explicit Compressor(uint8_t pcmID);
	Compressor();
	virtual ~Compressor();

	void Start();
	void Stop();
	bool Enabled();

	bool GetPressureSwitchValue();

	float GetCompressorCurrent();

	void SetClosedLoopControl(bool on);
	bool GetClosedLoopControl();

	bool GetCompressorCurrentTooHighFault();
	bool GetCompressorCurrentTooHighStickyFault();
	bool GetCompressorShortedStickyFault();
	bool GetCompressorShortedFault();
	bool GetCompressorNotConnectedStickyFault();
	bool GetCompressorNotConnectedFault();
	void ClearAllPCMStickyFaults();

protected:
	void *m_pcm_pointer;

private:
	void InitCompressor(uint8_t module);
	void SetCompressor(bool on);
};

#endif /* Compressor_H_ */
