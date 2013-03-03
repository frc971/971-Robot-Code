/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2009. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/


#ifndef CANJAGUAR_H
#define CANJAGUAR_H

#include "ErrorBase.h"
#include "MotorSafety.h"
#include "MotorSafetyHelper.h"
#include "PIDOutput.h"
#include "SpeedController.h"
#include <semLib.h>
#include <vxWorks.h>
#include "LiveWindow/LiveWindowSendable.h"
#include "tables/ITable.h"

/**
 * Luminary Micro Jaguar Speed Control
 */
class CANJaguar : public MotorSafety,
					public SpeedController,
					public ErrorBase,
					public LiveWindowSendable,
					public ITableListener
{
public:
	// The internal PID control loop in the Jaguar runs at 1kHz.
	static const INT32 kControllerRate = 1000;
	static const double kApproxBusVoltage = 12.0;

	typedef enum {kPercentVbus, kCurrent, kSpeed, kPosition, kVoltage} ControlMode;
	typedef enum {kCurrentFault = 1, kTemperatureFault = 2, kBusVoltageFault = 4, kGateDriverFault = 8} Faults;
	typedef enum {kForwardLimit = 1, kReverseLimit = 2} Limits;
	typedef enum {kPosRef_QuadEncoder = 0, kPosRef_Potentiometer = 1, kPosRef_None = 0xFF} PositionReference;
	typedef enum {kSpeedRef_Encoder = 0, kSpeedRef_InvEncoder = 2, kSpeedRef_QuadEncoder = 3, kSpeedRef_None = 0xFF} SpeedReference;
	typedef enum {kNeutralMode_Jumper = 0, kNeutralMode_Brake = 1, kNeutralMode_Coast = 2} NeutralMode;
	typedef enum {kLimitMode_SwitchInputsOnly = 0, kLimitMode_SoftPositionLimits = 1} LimitMode;

	explicit CANJaguar(UINT8 deviceNumber, ControlMode controlMode = kPercentVbus);
	virtual ~CANJaguar();

	// SpeedController interface
	virtual float Get();
	virtual void Set(float value, UINT8 syncGroup=0);
	virtual void Disable();

	// PIDOutput interface
	virtual void PIDWrite(float output);

	// Other Accessors
	void SetSpeedReference(SpeedReference reference);
	SpeedReference GetSpeedReference();
	void SetPositionReference(PositionReference reference);
	PositionReference GetPositionReference();
	void SetPID(double p, double i, double d);
	double GetP();
	double GetI();
	double GetD();
	void EnableControl(double encoderInitialPosition = 0.0);
	void DisableControl();
	void ChangeControlMode(ControlMode controlMode);
	ControlMode GetControlMode();
	float GetBusVoltage();
	float GetOutputVoltage();
	float GetOutputCurrent();
	float GetTemperature();
	double GetPosition();
	double GetSpeed();
	bool GetForwardLimitOK();
	bool GetReverseLimitOK();
	UINT16 GetFaults();
	bool GetPowerCycled();
	void SetVoltageRampRate(double rampRate);
	virtual UINT32 GetFirmwareVersion();
	UINT8 GetHardwareVersion();
	void ConfigNeutralMode(NeutralMode mode);
	void ConfigEncoderCodesPerRev(UINT16 codesPerRev);
	void ConfigPotentiometerTurns(UINT16 turns);
	void ConfigSoftPositionLimits(double forwardLimitPosition, double reverseLimitPosition);
	void DisableSoftPositionLimits();
	void ConfigMaxOutputVoltage(double voltage);
	void ConfigFaultTime(float faultTime);

	static void UpdateSyncGroup(UINT8 syncGroup);

	void SetExpiration(float timeout);
	float GetExpiration();
	bool IsAlive();
	void StopMotor();
	bool IsSafetyEnabled();
	void SetSafetyEnabled(bool enabled);
	void GetDescription(char *desc);

protected:
	UINT8 packPercentage(UINT8 *buffer, double value);
	UINT8 packFXP8_8(UINT8 *buffer, double value);
	UINT8 packFXP16_16(UINT8 *buffer, double value);
	UINT8 packINT16(UINT8 *buffer, INT16 value);
	UINT8 packINT32(UINT8 *buffer, INT32 value);
	double unpackPercentage(UINT8 *buffer);
	double unpackFXP8_8(UINT8 *buffer);
	double unpackFXP16_16(UINT8 *buffer);
	INT16 unpackINT16(UINT8 *buffer);
	INT32 unpackINT32(UINT8 *buffer);
	virtual void setTransaction(UINT32 messageID, const UINT8 *data, UINT8 dataSize);
	virtual void getTransaction(UINT32 messageID, UINT8 *data, UINT8 *dataSize);

	static INT32 sendMessage(UINT32 messageID, const UINT8 *data, UINT8 dataSize);
	static INT32 receiveMessage(UINT32 *messageID, UINT8 *data, UINT8 *dataSize, float timeout = 0.02);

	UINT8 m_deviceNumber;
	ControlMode m_controlMode;
	SEM_ID m_transactionSemaphore;
	double m_maxOutputVoltage;

	MotorSafetyHelper *m_safetyHelper;

	void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew);
	void UpdateTable();
	void StartLiveWindowMode();
	void StopLiveWindowMode();
	std::string GetSmartDashboardType();
	void InitTable(ITable *subTable);
	ITable * GetTable();
	
	ITable *m_table;

private:
	void InitCANJaguar();
};
#endif

