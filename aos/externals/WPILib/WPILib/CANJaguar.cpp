/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2009. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "CANJaguar.h"
#define tNIRIO_i32 int
#include "ChipObject/NiFpga.h"
#include "CAN/JaguarCANDriver.h"
#include "CAN/can_proto.h"
#include "NetworkCommunication/UsageReporting.h"
#include "WPIErrors.h"
#include <stdio.h>
#include "LiveWindow/LiveWindow.h"

#define swap16(x) ( (((x)>>8) &0x00FF) \
                  | (((x)<<8) &0xFF00) )
#define swap32(x) ( (((x)>>24)&0x000000FF) \
                  | (((x)>>8) &0x0000FF00) \
                  | (((x)<<8) &0x00FF0000) \
                  | (((x)<<24)&0xFF000000) )

#define kFullMessageIDMask (CAN_MSGID_API_M | CAN_MSGID_MFR_M | CAN_MSGID_DTYPE_M)

const INT32 CANJaguar::kControllerRate;
const double CANJaguar::kApproxBusVoltage;

/**
 * Common initialization code called by all constructors.
 */
void CANJaguar::InitCANJaguar()
{
	m_transactionSemaphore = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);
	if (m_deviceNumber < 1 || m_deviceNumber > 63)
	{
		char buf[256];
		snprintf(buf, 256, "device number \"%d\" must be between 1 and 63", m_deviceNumber);
		wpi_setWPIErrorWithContext(ParameterOutOfRange, buf);
		return;
	}
	UINT32 fwVer = GetFirmwareVersion();
	if (StatusIsFatal())
		return;
	// 3330 was the first shipping RDK firmware version for the Jaguar
	if (fwVer >= 3330 || fwVer < 101)
	{
		char buf[256];
		if (fwVer < 3330)
		{
			snprintf(buf, 256, "Jag #%d firmware (%d) is too old (must be at least version 101 of the FIRST approved firmware)", m_deviceNumber, fwVer);
		}
		else
		{
			snprintf(buf, 256, "Jag #%d firmware (%d) is not FIRST approved (must be at least version 101 of the FIRST approved firmware)", m_deviceNumber, fwVer);
		}
		wpi_setWPIErrorWithContext(JaguarVersionError, buf);
		return;
	}
	switch (m_controlMode)
	{
	case kPercentVbus:
	case kVoltage:
		// No additional configuration required... start enabled.
		EnableControl();
		break;
	default:
		break;
	}
	m_safetyHelper = new MotorSafetyHelper(this);

	nUsageReporting::report(nUsageReporting::kResourceType_CANJaguar, m_deviceNumber, m_controlMode);
	LiveWindow::GetInstance()->AddActuator("CANJaguar", m_deviceNumber, 0, this);
}

/**
 * Constructor
 * 
 * @param deviceNumber The the address of the Jaguar on the CAN bus.
 */
CANJaguar::CANJaguar(UINT8 deviceNumber, ControlMode controlMode)
	: m_deviceNumber (deviceNumber)
	, m_controlMode (controlMode)
	, m_transactionSemaphore (NULL)
	, m_maxOutputVoltage (kApproxBusVoltage)
	, m_safetyHelper (NULL)
{
	InitCANJaguar();
}

CANJaguar::~CANJaguar()
{
	delete m_safetyHelper;
	m_safetyHelper = NULL;
	semDelete(m_transactionSemaphore);
	m_transactionSemaphore = NULL;
}

/**
 * Set the output set-point value.  
 * 
 * The scale and the units depend on the mode the Jaguar is in.
 * In PercentVbus Mode, the outputValue is from -1.0 to 1.0 (same as PWM Jaguar).
 * In Voltage Mode, the outputValue is in Volts.
 * In Current Mode, the outputValue is in Amps.
 * In Speed Mode, the outputValue is in Rotations/Minute.
 * In Position Mode, the outputValue is in Rotations.
 * 
 * @param outputValue The set-point to sent to the motor controller.
 * @param syncGroup The update group to add this Set() to, pending UpdateSyncGroup().  If 0, update immediately.
 */
void CANJaguar::Set(float outputValue, UINT8 syncGroup)
{
	UINT32 messageID;
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	if (m_safetyHelper && !m_safetyHelper->IsAlive())
	{
		EnableControl();
	}

	switch(m_controlMode)
	{
	case kPercentVbus:
		{
			messageID = LM_API_VOLT_T_SET;
			if (outputValue > 1.0) outputValue = 1.0;
			if (outputValue < -1.0) outputValue = -1.0;
			dataSize = packPercentage(dataBuffer, outputValue);
		}
		break;
	case kSpeed:
		{
			messageID = LM_API_SPD_T_SET;
			dataSize = packFXP16_16(dataBuffer, outputValue);
		}
		break;
	case kPosition:
		{
			messageID = LM_API_POS_T_SET;
			dataSize = packFXP16_16(dataBuffer, outputValue);
		}
		break;
	case kCurrent:
		{
			messageID = LM_API_ICTRL_T_SET;
			dataSize = packFXP8_8(dataBuffer, outputValue);
		}
		break;
	case kVoltage:
		{
			messageID = LM_API_VCOMP_T_SET;
			dataSize = packFXP8_8(dataBuffer, outputValue);
		}
		break;
	default:
		return;
	}
	if (syncGroup != 0)
	{
		dataBuffer[dataSize] = syncGroup;
		dataSize++;
	}
	setTransaction(messageID, dataBuffer, dataSize);
	if (m_safetyHelper) m_safetyHelper->Feed();
}

/**
 * Get the recently set outputValue setpoint.
 * 
 * The scale and the units depend on the mode the Jaguar is in.
 * In PercentVbus Mode, the outputValue is from -1.0 to 1.0 (same as PWM Jaguar).
 * In Voltage Mode, the outputValue is in Volts.
 * In Current Mode, the outputValue is in Amps.
 * In Speed Mode, the outputValue is in Rotations/Minute.
 * In Position Mode, the outputValue is in Rotations.
 * 
 * @return The most recently set outputValue setpoint.
 */
float CANJaguar::Get()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	switch(m_controlMode)
	{
	case kPercentVbus:
		getTransaction(LM_API_VOLT_SET, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT16))
		{
			return unpackPercentage(dataBuffer);
		}
		break;
	case kSpeed:
		getTransaction(LM_API_SPD_SET, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	case kPosition:
		getTransaction(LM_API_POS_SET, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	case kCurrent:
		getTransaction(LM_API_ICTRL_SET, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT16))
		{
			return unpackFXP8_8(dataBuffer);
		}
		break;
	case kVoltage:
		getTransaction(LM_API_VCOMP_SET, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT16))
		{
			return unpackFXP8_8(dataBuffer);
		}
		break;
	}
	return 0.0;
}

/**
 * Common interface for disabling a motor.
 * 
 * @deprecated Call DisableControl instead.
 */
void CANJaguar::Disable()
{
	DisableControl();
}

/**
 * Write out the PID value as seen in the PIDOutput base object.
 * 
 * @deprecated Call Set instead.
 * 
 * @param output Write out the PercentVbus value as was computed by the PIDController
 */
void CANJaguar::PIDWrite(float output)
{
	if (m_controlMode == kPercentVbus)
	{
		Set(output);
	}
	else
	{
		wpi_setWPIErrorWithContext(IncompatibleMode, "PID only supported in PercentVbus mode");
	}
}

UINT8 CANJaguar::packPercentage(UINT8 *buffer, double value)
{
	INT16 intValue = (INT16)(value * 32767.0);
	*((INT16*)buffer) = swap16(intValue);
	return sizeof(INT16);
}

UINT8 CANJaguar::packFXP8_8(UINT8 *buffer, double value)
{
	INT16 intValue = (INT16)(value * 256.0);
	*((INT16*)buffer) = swap16(intValue);
	return sizeof(INT16);
}

UINT8 CANJaguar::packFXP16_16(UINT8 *buffer, double value)
{
	INT32 intValue = (INT32)(value * 65536.0);
	*((INT32*)buffer) = swap32(intValue);
	return sizeof(INT32);
}

UINT8 CANJaguar::packINT16(UINT8 *buffer, INT16 value)
{
	*((INT16*)buffer) = swap16(value);
	return sizeof(INT16);
}

UINT8 CANJaguar::packINT32(UINT8 *buffer, INT32 value)
{
	*((INT32*)buffer) = swap32(value);
	return sizeof(INT32);
}

double CANJaguar::unpackPercentage(UINT8 *buffer)
{
	INT16 value = *((INT16*)buffer);
	value = swap16(value);
	return value / 32767.0;
}

double CANJaguar::unpackFXP8_8(UINT8 *buffer)
{
	INT16 value = *((INT16*)buffer);
	value = swap16(value);
	return value / 256.0;
}

double CANJaguar::unpackFXP16_16(UINT8 *buffer)
{
	INT32 value = *((INT32*)buffer);
	value = swap32(value);
	return value / 65536.0;
}

INT16 CANJaguar::unpackINT16(UINT8 *buffer)
{
	INT16 value = *((INT16*)buffer);
	return swap16(value);
}

INT32 CANJaguar::unpackINT32(UINT8 *buffer)
{
	INT32 value = *((INT32*)buffer);
	return swap32(value);
}

/**
 * Send a message on the CAN bus through the CAN driver in FRC_NetworkCommunication
 * 
 * Trusted messages require a 2-byte token at the beginning of the data payload.
 * If the message being sent is trusted, make space for the token.
 * 
 * @param messageID The messageID to be used on the CAN bus
 * @param data The up to 8 bytes of data to be sent with the message
 * @param dataSize Specify how much of the data in "data" to send
 * @return Status of send call
 */
INT32 CANJaguar::sendMessage(UINT32 messageID, const UINT8 *data, UINT8 dataSize)
{
	static const UINT32 kTrustedMessages[] = {
			LM_API_VOLT_T_EN, LM_API_VOLT_T_SET, LM_API_SPD_T_EN, LM_API_SPD_T_SET,
			LM_API_VCOMP_T_EN, LM_API_VCOMP_T_SET, LM_API_POS_T_EN, LM_API_POS_T_SET,
			LM_API_ICTRL_T_EN, LM_API_ICTRL_T_SET};
	INT32 status=0;

	for (UINT8 i=0; i<(sizeof(kTrustedMessages)/sizeof(kTrustedMessages[0])); i++)
	{
		if ((kFullMessageIDMask & messageID) == kTrustedMessages[i])
		{
			UINT8 dataBuffer[8];
			dataBuffer[0] = 0;
			dataBuffer[1] = 0;
			// Make sure the data will still fit after adjusting for the token.
			if (dataSize > 6)
			{
				// TODO: I would rather this not have to set the global error
				wpi_setGlobalWPIErrorWithContext(ParameterOutOfRange, "dataSize > 6");
				return 0;
			}
			for (UINT8 j=0; j < dataSize; j++)
			{
				dataBuffer[j + 2] = data[j];
			}
			FRC_NetworkCommunication_JaguarCANDriver_sendMessage(messageID, dataBuffer, dataSize + 2, &status);
			return status;
		}
	}
	FRC_NetworkCommunication_JaguarCANDriver_sendMessage(messageID, data, dataSize, &status);
	return status;
}

/**
 * Receive a message from the CAN bus through the CAN driver in FRC_NetworkCommunication
 * 
 * @param messageID The messageID to read from the CAN bus
 * @param data The up to 8 bytes of data that was received with the message
 * @param dataSize Indicates how much data was received
 * @param timeout Specify how long to wait for a message (in seconds)
 * @return Status of receive call
 */
INT32 CANJaguar::receiveMessage(UINT32 *messageID, UINT8 *data, UINT8 *dataSize, float timeout)
{
	INT32 status = 0;
	FRC_NetworkCommunication_JaguarCANDriver_receiveMessage(messageID, data, dataSize,
			(UINT32)(timeout * 1000), &status);
	return status;
}

/**
 * Execute a transaction with a Jaguar that sets some property.
 * 
 * Jaguar always acks when it receives a message.  If we don't wait for an ack,
 * the message object in the Jaguar could get overwritten before it is handled.
 * 
 * @param messageID The messageID to be used on the CAN bus (device number is added internally)
 * @param data The up to 8 bytes of data to be sent with the message
 * @param dataSize Specify how much of the data in "data" to send
 */
void CANJaguar::setTransaction(UINT32 messageID, const UINT8 *data, UINT8 dataSize)
{
	UINT32 ackMessageID = LM_API_ACK | m_deviceNumber;
	INT32 localStatus = 0;

	// If there was an error on this object and it wasn't a timeout, refuse to talk to the device
	// Call ClearError() on the object to try again
	if (StatusIsFatal() && GetError().GetCode() != -44087)
		return;

	// Make sure we don't have more than one transaction with the same Jaguar outstanding.
	semTake(m_transactionSemaphore, WAIT_FOREVER);

	// Throw away any stale acks.
	receiveMessage(&ackMessageID, NULL, 0, 0.0f);
	// Send the message with the data.
	localStatus = sendMessage(messageID | m_deviceNumber, data, dataSize);
	wpi_setErrorWithContext(localStatus, "sendMessage");
	// Wait for an ack.
	localStatus = receiveMessage(&ackMessageID, NULL, 0);
	wpi_setErrorWithContext(localStatus, "receiveMessage");

	// Transaction complete.
	semGive(m_transactionSemaphore);
}

/**
 * Execute a transaction with a Jaguar that gets some property.
 * 
 * Jaguar always generates a message with the same message ID when replying.
 * 
 * @param messageID The messageID to read from the CAN bus (device number is added internally)
 * @param data The up to 8 bytes of data that was received with the message
 * @param dataSize Indicates how much data was received
 */
void CANJaguar::getTransaction(UINT32 messageID, UINT8 *data, UINT8 *dataSize)
{
	UINT32 targetedMessageID = messageID | m_deviceNumber;
	INT32 localStatus = 0;

	// If there was an error on this object and it wasn't a timeout, refuse to talk to the device
	// Call ClearError() on the object to try again
	if (StatusIsFatal() && GetError().GetCode() != -44087)
	{
		if (dataSize != NULL)
			*dataSize = 0;
		return;
	}

	// Make sure we don't have more than one transaction with the same Jaguar outstanding.
	semTake(m_transactionSemaphore, WAIT_FOREVER);

	// Throw away any stale responses.
	receiveMessage(&targetedMessageID, NULL, 0, 0.0f);
	// Send the message requesting data.
	localStatus = sendMessage(targetedMessageID, NULL, 0);
	wpi_setErrorWithContext(localStatus, "sendMessage");
	// Caller may have set bit31 for remote frame transmission so clear invalid bits[31-29]
	targetedMessageID &= 0x1FFFFFFF;
	// Wait for the data.
	localStatus = receiveMessage(&targetedMessageID, data, dataSize);
	wpi_setErrorWithContext(localStatus, "receiveMessage");

	// Transaction complete.
	semGive(m_transactionSemaphore);
}

/**
 * Set the reference source device for speed controller mode.
 * 
 * Choose encoder as the source of speed feedback when in speed control mode.
 * 
 * @param reference Specify a SpeedReference.
 */
void CANJaguar::SetSpeedReference(SpeedReference reference)
{
	UINT8 dataBuffer[8];

	dataBuffer[0] = reference;
	setTransaction(LM_API_SPD_REF, dataBuffer, sizeof(UINT8));
}

/**
 * Get the reference source device for speed controller mode.
 * 
 * @return A SpeedReference indicating the currently selected reference device for speed controller mode.
 */
CANJaguar::SpeedReference CANJaguar::GetSpeedReference()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_SPD_REF, dataBuffer, &dataSize);
	if (dataSize == sizeof(UINT8))
	{
		return (SpeedReference)*dataBuffer;
	}
	return kSpeedRef_None;
}

/**
 * Set the reference source device for position controller mode.
 * 
 * Choose between using and encoder and using a potentiometer
 * as the source of position feedback when in position control mode.
 * 
 * @param reference Specify a PositionReference.
 */
void CANJaguar::SetPositionReference(PositionReference reference)
{
	UINT8 dataBuffer[8];

	dataBuffer[0] = reference;
	setTransaction(LM_API_POS_REF, dataBuffer, sizeof(UINT8));
}

/**
 * Get the reference source device for position controller mode.
 * 
 * @return A PositionReference indicating the currently selected reference device for position controller mode.
 */
CANJaguar::PositionReference CANJaguar::GetPositionReference()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_POS_REF, dataBuffer, &dataSize);
	if (dataSize == sizeof(UINT8))
	{
		return (PositionReference)*dataBuffer;
	}
	return kPosRef_None;
}

/**
 * Set the P, I, and D constants for the closed loop modes.
 * 
 * @param p The proportional gain of the Jaguar's PID controller.
 * @param i The integral gain of the Jaguar's PID controller.
 * @param d The differential gain of the Jaguar's PID controller.
 */
void CANJaguar::SetPID(double p, double i, double d)
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	switch(m_controlMode)
	{
	case kPercentVbus:
	case kVoltage:
		wpi_setWPIErrorWithContext(IncompatibleMode, "PID constants only apply in Speed, Position, and Current mode");
		break;
	case kSpeed:
		dataSize = packFXP16_16(dataBuffer, p);
		setTransaction(LM_API_SPD_PC, dataBuffer, dataSize);
		dataSize = packFXP16_16(dataBuffer, i);
		setTransaction(LM_API_SPD_IC, dataBuffer, dataSize);
		dataSize = packFXP16_16(dataBuffer, d);
		setTransaction(LM_API_SPD_DC, dataBuffer, dataSize);
		break;
	case kPosition:
		dataSize = packFXP16_16(dataBuffer, p);
		setTransaction(LM_API_POS_PC, dataBuffer, dataSize);
		dataSize = packFXP16_16(dataBuffer, i);
		setTransaction(LM_API_POS_IC, dataBuffer, dataSize);
		dataSize = packFXP16_16(dataBuffer, d);
		setTransaction(LM_API_POS_DC, dataBuffer, dataSize);
		break;
	case kCurrent:
		dataSize = packFXP16_16(dataBuffer, p);
		setTransaction(LM_API_ICTRL_PC, dataBuffer, dataSize);
		dataSize = packFXP16_16(dataBuffer, i);
		setTransaction(LM_API_ICTRL_IC, dataBuffer, dataSize);
		dataSize = packFXP16_16(dataBuffer, d);
		setTransaction(LM_API_ICTRL_DC, dataBuffer, dataSize);
		break;
	}
}

/**
 * Get the Proportional gain of the controller.
 * 
 * @return The proportional gain.
 */
double CANJaguar::GetP()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	switch(m_controlMode)
	{
	case kPercentVbus:
	case kVoltage:
		wpi_setWPIErrorWithContext(IncompatibleMode, "PID constants only apply in Speed, Position, and Current mode");
		break;
	case kSpeed:
		getTransaction(LM_API_SPD_PC, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	case kPosition:
		getTransaction(LM_API_POS_PC, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	case kCurrent:
		getTransaction(LM_API_ICTRL_PC, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	}
	return 0.0;
}

/**
 * Get the Intregral gain of the controller.
 * 
 * @return The integral gain.
 */
double CANJaguar::GetI()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	switch(m_controlMode)
	{
	case kPercentVbus:
	case kVoltage:
		wpi_setWPIErrorWithContext(IncompatibleMode, "PID constants only apply in Speed, Position, and Current mode");
		break;
	case kSpeed:
		getTransaction(LM_API_SPD_IC, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	case kPosition:
		getTransaction(LM_API_POS_IC, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	case kCurrent:
		getTransaction(LM_API_ICTRL_IC, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	}
	return 0.0;
}

/**
 * Get the Differential gain of the controller.
 * 
 * @return The differential gain.
 */
double CANJaguar::GetD()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	switch(m_controlMode)
	{
	case kPercentVbus:
	case kVoltage:
		wpi_setWPIErrorWithContext(IncompatibleMode, "PID constants only apply in Speed, Position, and Current mode");
		break;
	case kSpeed:
		getTransaction(LM_API_SPD_DC, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	case kPosition:
		getTransaction(LM_API_POS_DC, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	case kCurrent:
		getTransaction(LM_API_ICTRL_DC, dataBuffer, &dataSize);
		if (dataSize == sizeof(INT32))
		{
			return unpackFXP16_16(dataBuffer);
		}
		break;
	}
	return 0.0;
}

/**
 * Enable the closed loop controller.
 * 
 * Start actually controlling the output based on the feedback.
 * If starting a position controller with an encoder reference,
 * use the encoderInitialPosition parameter to initialize the
 * encoder state.
 * 
 * @param encoderInitialPosition Encoder position to set if position with encoder reference.  Ignored otherwise.
 */
void CANJaguar::EnableControl(double encoderInitialPosition)
{
	UINT8 dataBuffer[8];
	UINT8 dataSize = 0;

	switch(m_controlMode)
	{
	case kPercentVbus:
		setTransaction(LM_API_VOLT_T_EN, dataBuffer, dataSize);
		break;
	case kSpeed:
		setTransaction(LM_API_SPD_T_EN, dataBuffer, dataSize);
		break;
	case kPosition:
		dataSize = packFXP16_16(dataBuffer, encoderInitialPosition);
		setTransaction(LM_API_POS_T_EN, dataBuffer, dataSize);
		break;
	case kCurrent:
		setTransaction(LM_API_ICTRL_T_EN, dataBuffer, dataSize);
		break;
	case kVoltage:
		setTransaction(LM_API_VCOMP_T_EN, dataBuffer, dataSize);
		break;
	}
}

/**
 * Disable the closed loop controller.
 * 
 * Stop driving the output based on the feedback.
 */
void CANJaguar::DisableControl()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize = 0;

	switch(m_controlMode)
	{
	case kPercentVbus:
		setTransaction(LM_API_VOLT_DIS, dataBuffer, dataSize);
		break;
	case kSpeed:
		setTransaction(LM_API_SPD_DIS, dataBuffer, dataSize);
		break;
	case kPosition:
		setTransaction(LM_API_POS_DIS, dataBuffer, dataSize);
		break;
	case kCurrent:
		setTransaction(LM_API_ICTRL_DIS, dataBuffer, dataSize);
		break;
	case kVoltage:
		setTransaction(LM_API_VCOMP_DIS, dataBuffer, dataSize);
		break;
	}
}

/**
 * Change the control mode of this Jaguar object.
 * 
 * After changing modes, configure any PID constants or other settings needed
 * and then EnableControl() to actually change the mode on the Jaguar.
 * 
 * @param controlMode The new mode.
 */
void CANJaguar::ChangeControlMode(ControlMode controlMode)
{
	// Disable the previous mode
	DisableControl();

	// Update the local mode
	m_controlMode = controlMode;

	nUsageReporting::report(nUsageReporting::kResourceType_CANJaguar, m_deviceNumber, m_controlMode);
}

/**
 * Get the active control mode from the Jaguar.
 * 
 * Ask the Jag what mode it is in.
 * 
 * @return ControlMode that the Jag is in.
 */
CANJaguar::ControlMode CANJaguar::GetControlMode()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_CMODE, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT8))
	{
		return (ControlMode)dataBuffer[0];
	}
	return kPercentVbus;
}

/**
 * Get the voltage at the battery input terminals of the Jaguar.
 * 
 * @return The bus voltage in Volts.
 */
float CANJaguar::GetBusVoltage()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_VOLTBUS, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT16))
	{
		return unpackFXP8_8(dataBuffer);
	}
	return 0.0;
}

/**
 * Get the voltage being output from the motor terminals of the Jaguar.
 * 
 * @return The output voltage in Volts.
 */
float CANJaguar::GetOutputVoltage()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	// Read the volt out which is in Volts units.
	getTransaction(LM_API_STATUS_VOUT, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT16))
	{
		return unpackFXP8_8(dataBuffer);
	}
	return 0.0;
}

/**
 * Get the current through the motor terminals of the Jaguar.
 * 
 * @return The output current in Amps.
 */
float CANJaguar::GetOutputCurrent()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_CURRENT, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT16))
	{
		return unpackFXP8_8(dataBuffer);
	}
	return 0.0;
}

/**
 * Get the internal temperature of the Jaguar.
 * 
 * @return The temperature of the Jaguar in degrees Celsius.
 */
float CANJaguar::GetTemperature()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_TEMP, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT16))
	{
		return unpackFXP8_8(dataBuffer);
	}
	return 0.0;
}

/**
 * Get the position of the encoder or potentiometer.
 * 
 * @return The position of the motor in rotations based on the configured feedback.
 */
double CANJaguar::GetPosition()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_POS, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT32))
	{
		return unpackFXP16_16(dataBuffer);
	}
	return 0.0;
}

/**
 * Get the speed of the encoder.
 * 
 * @return The speed of the motor in RPM based on the configured feedback.
 */
double CANJaguar::GetSpeed()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_SPD, dataBuffer, &dataSize);
	if (dataSize == sizeof(INT32))
	{
		return unpackFXP16_16(dataBuffer);
	}
	return 0.0;
}

/**
 * Get the status of the forward limit switch.
 * 
 * @return The motor is allowed to turn in the forward direction when true.
 */
bool CANJaguar::GetForwardLimitOK()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_LIMIT, dataBuffer, &dataSize);
	if (dataSize == sizeof(UINT8))
	{
		return (*dataBuffer & kForwardLimit) != 0;
	}
	return 0;
}

/**
 * Get the status of the reverse limit switch.
 * 
 * @return The motor is allowed to turn in the reverse direction when true.
 */
bool CANJaguar::GetReverseLimitOK()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_LIMIT, dataBuffer, &dataSize);
	if (dataSize == sizeof(UINT8))
	{
		return (*dataBuffer & kReverseLimit) != 0;
	}
	return 0;
}

/**
 * Get the status of any faults the Jaguar has detected.
 * 
 * @return A bit-mask of faults defined by the "Faults" enum.
 */
UINT16 CANJaguar::GetFaults()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_FAULT, dataBuffer, &dataSize);
	if (dataSize == sizeof(UINT16))
	{
		return unpackINT16(dataBuffer);
	}
	return 0;
}

/**
 * Check if the Jaguar's power has been cycled since this was last called.
 * 
 * This should return true the first time called after a Jaguar power up,
 * and false after that.
 * 
 * @return The Jaguar was power cycled since the last call to this function.
 */
bool CANJaguar::GetPowerCycled()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_STATUS_POWER, dataBuffer, &dataSize);
	if (dataSize == sizeof(UINT8))
	{
		bool powerCycled = (*dataBuffer != 0);

		// Clear the power cycled bit now that we've accessed it
		if (powerCycled)
		{
			dataBuffer[0] = 1;
			setTransaction(LM_API_STATUS_POWER, dataBuffer, sizeof(UINT8));
		}

		return powerCycled;
	}
	return 0;
}

/**
 * Set the maximum voltage change rate.
 * 
 * When in PercentVbus or Voltage output mode, the rate at which the voltage changes can
 * be limited to reduce current spikes.  Set this to 0.0 to disable rate limiting.
 * 
 * @param rampRate The maximum rate of voltage change in Percent Voltage mode in V/s.
 */
void CANJaguar::SetVoltageRampRate(double rampRate)
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	switch(m_controlMode)
	{
	case kPercentVbus:
		dataSize = packPercentage(dataBuffer, rampRate / (m_maxOutputVoltage * kControllerRate));
		setTransaction(LM_API_VOLT_SET_RAMP, dataBuffer, dataSize);
		break;
	case kVoltage:
		dataSize = packFXP8_8(dataBuffer, rampRate / kControllerRate);
		setTransaction(LM_API_VCOMP_IN_RAMP, dataBuffer, dataSize);
		break;
	default:
		return;
	}
}

/**
 * Get the version of the firmware running on the Jaguar.
 * 
 * @return The firmware version.  0 if the device did not respond.
 */
UINT32 CANJaguar::GetFirmwareVersion()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	// Set the MSB to tell the 2CAN that this is a remote message.
	getTransaction(0x80000000 | CAN_MSGID_API_FIRMVER, dataBuffer, &dataSize);
	if (dataSize == sizeof(UINT32))
	{
		return unpackINT32(dataBuffer);
	}
	return 0;
}

/**
 * Get the version of the Jaguar hardware.
 * 
 * @return The hardware version. 1: Jaguar,  2: Black Jaguar
 */
UINT8 CANJaguar::GetHardwareVersion()
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	getTransaction(LM_API_HWVER, dataBuffer, &dataSize);
	if (dataSize == sizeof(UINT8)+sizeof(UINT8))
	{
		if (*dataBuffer == m_deviceNumber)
		{
			return *(dataBuffer+1);
		}
	}
	// Assume Gray Jag if there is no response
	return LM_HWVER_JAG_1_0;
}

/**
 * Configure what the controller does to the H-Bridge when neutral (not driving the output).
 * 
 * This allows you to override the jumper configuration for brake or coast.
 * 
 * @param mode Select to use the jumper setting or to override it to coast or brake.
 */
void CANJaguar::ConfigNeutralMode(NeutralMode mode)
{
	UINT8 dataBuffer[8];

	dataBuffer[0] = mode;
	setTransaction(LM_API_CFG_BRAKE_COAST, dataBuffer, sizeof(UINT8));
}

/**
 * Configure how many codes per revolution are generated by your encoder.
 * 
 * @param codesPerRev The number of counts per revolution in 1X mode.
 */
void CANJaguar::ConfigEncoderCodesPerRev(UINT16 codesPerRev)
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	dataSize = packINT16(dataBuffer, codesPerRev);
	setTransaction(LM_API_CFG_ENC_LINES, dataBuffer, dataSize);
}

/**
 * Configure the number of turns on the potentiometer.
 * 
 * There is no special support for continuous turn potentiometers.
 * Only integer numbers of turns are supported.
 * 
 * @param turns The number of turns of the potentiometer
 */
void CANJaguar::ConfigPotentiometerTurns(UINT16 turns)
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	dataSize = packINT16(dataBuffer, turns);
	setTransaction(LM_API_CFG_POT_TURNS, dataBuffer, dataSize);
}

/**
 * Configure Soft Position Limits when in Position Controller mode.
 * 
 * When controlling position, you can add additional limits on top of the limit switch inputs
 * that are based on the position feedback.  If the position limit is reached or the
 * switch is opened, that direction will be disabled.
 * 
 * @param forwardLimitPosition The position that if exceeded will disable the forward direction.
 * @param reverseLimitPosition The position that if exceeded will disable the reverse direction.
 */
void CANJaguar::ConfigSoftPositionLimits(double forwardLimitPosition, double reverseLimitPosition)
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	dataSize = packFXP16_16(dataBuffer, forwardLimitPosition);
	dataBuffer[dataSize++] = forwardLimitPosition > reverseLimitPosition;
	setTransaction(LM_API_CFG_LIMIT_FWD, dataBuffer, dataSize);

	dataSize = packFXP16_16(dataBuffer, reverseLimitPosition);
	dataBuffer[dataSize++] = forwardLimitPosition <= reverseLimitPosition;
	setTransaction(LM_API_CFG_LIMIT_REV, dataBuffer, dataSize);

	dataBuffer[0] = kLimitMode_SoftPositionLimits;
	setTransaction(LM_API_CFG_LIMIT_MODE, dataBuffer, sizeof(UINT8));
}

/**
 * Disable Soft Position Limits if previously enabled.
 * 
 * Soft Position Limits are disabled by default.
 */
void CANJaguar::DisableSoftPositionLimits()
{
	UINT8 dataBuffer[8];

	dataBuffer[0] = kLimitMode_SwitchInputsOnly;
	setTransaction(LM_API_CFG_LIMIT_MODE, dataBuffer, sizeof(UINT8));
}

/**
 * Configure the maximum voltage that the Jaguar will ever output.
 * 
 * This can be used to limit the maximum output voltage in all modes so that
 * motors which cannot withstand full bus voltage can be used safely.
 * 
 * @param voltage The maximum voltage output by the Jaguar.
 */
void CANJaguar::ConfigMaxOutputVoltage(double voltage)
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	m_maxOutputVoltage = voltage;
	dataSize = packFXP8_8(dataBuffer, voltage);
	setTransaction(LM_API_CFG_MAX_VOUT, dataBuffer, dataSize);
}

/**
 * Configure how long the Jaguar waits in the case of a fault before resuming operation.
 * 
 * Faults include over temerature, over current, and bus under voltage.
 * The default is 3.0 seconds, but can be reduced to as low as 0.5 seconds.
 * 
 * @param faultTime The time to wait before resuming operation, in seconds.
 */
void CANJaguar::ConfigFaultTime(float faultTime)
{
	UINT8 dataBuffer[8];
	UINT8 dataSize;

	// Message takes ms
	dataSize = packINT16(dataBuffer, (INT16)(faultTime * 1000.0));
	setTransaction(LM_API_CFG_FAULT_TIME, dataBuffer, dataSize);
}

/**
 * Update all the motors that have pending sets in the syncGroup.
 * 
 * @param syncGroup A bitmask of groups to generate synchronous output.
 */
void CANJaguar::UpdateSyncGroup(UINT8 syncGroup)
{
	sendMessage(CAN_MSGID_API_SYNC, &syncGroup, sizeof(syncGroup));
}


void CANJaguar::SetExpiration(float timeout)
{
	if (m_safetyHelper) m_safetyHelper->SetExpiration(timeout);
}

float CANJaguar::GetExpiration()
{
	if (!m_safetyHelper) return 0.0;
	return m_safetyHelper->GetExpiration();
}

bool CANJaguar::IsAlive()
{
	if (!m_safetyHelper) return false;
	return m_safetyHelper->IsAlive();
}

bool CANJaguar::IsSafetyEnabled()
{
	if (!m_safetyHelper) return false;
	return m_safetyHelper->IsSafetyEnabled();
}

void CANJaguar::SetSafetyEnabled(bool enabled)
{
	if (m_safetyHelper) m_safetyHelper->SetSafetyEnabled(enabled);
}

void CANJaguar::GetDescription(char *desc)
{
	sprintf(desc, "CANJaguar ID %d", m_deviceNumber);
}

/**
 * Common interface for stopping the motor
 * Part of the MotorSafety interface
 * 
 * @deprecated Call DisableControl instead.
 */
void CANJaguar::StopMotor()
{
	DisableControl();
}

void CANJaguar::ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew) {
	Set(value.f);
}

void CANJaguar::UpdateTable() {
	if (m_table != NULL) {
		m_table->PutNumber("Value", Get());
	}
}

void CANJaguar::StartLiveWindowMode() {
	m_table->AddTableListener("Value", this, true);
}

void CANJaguar::StopLiveWindowMode() {
	m_table->RemoveTableListener(this);
}

std::string CANJaguar::GetSmartDashboardType() {
	return "Speed Controller";
}

void CANJaguar::InitTable(ITable *subTable) {
	m_table = subTable;
	UpdateTable();
}

ITable * CANJaguar::GetTable() {
	return m_table;
}


