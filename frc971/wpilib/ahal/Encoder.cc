/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc971/wpilib/ahal/Encoder.h"

#include "hal/HAL.h"
#include "frc971/wpilib/ahal/DigitalInput.h"

using namespace frc;

#define HAL_FATAL_WITH_STATUS(status)

/**
 * Common initialization code for Encoders.
 *
 * This code allocates resources for Encoders and is common to all constructors.
 *
 * The counter will start counting immediately.
 *
 * @param reverseDirection If true, counts down instead of up (this is all
 *                         relative)
 * @param encodingType     either k1X, k2X, or k4X to indicate 1X, 2X or 4X
 *                         decoding. If 4X is selected, then an encoder FPGA
 *                         object is used and the returned counts will be 4x
 *                         the encoder spec'd value since all rising and
 *                         falling edges are counted. If 1X or 2X are selected
 *                         then a counter object will be used and the returned
 *                         value will either exactly match the spec'd count or
 *                         be double (2x) the spec'd count.
 */
void Encoder::InitEncoder(bool reverseDirection, EncodingType encodingType) {
  int32_t status = 0;
  m_encoder = HAL_InitializeEncoder(
      m_aSource->GetPortHandleForRouting(),
      (HAL_AnalogTriggerType)m_aSource->GetAnalogTriggerTypeForRouting(),
      m_bSource->GetPortHandleForRouting(),
      (HAL_AnalogTriggerType)m_bSource->GetAnalogTriggerTypeForRouting(),
      reverseDirection, (HAL_EncoderEncodingType)encodingType, &status);
  HAL_FATAL_WITH_STATUS(status);

  HAL_Report(HALUsageReporting::kResourceType_Encoder, GetFPGAIndex(),
             encodingType);
}

/**
 * Encoder constructor.
 *
 * Construct a Encoder given a and b channels.
 *
 * The counter will start counting immediately.
 *
 * @param aChannel         The a channel DIO channel. 0-9 are on-board, 10-25
 *                         are on the MXP port
 * @param bChannel         The b channel DIO channel. 0-9 are on-board, 10-25
 *                         are on the MXP port
 * @param reverseDirection represents the orientation of the encoder and
 *                         inverts the output values if necessary so forward
 *                         represents positive values.
 * @param encodingType     either k1X, k2X, or k4X to indicate 1X, 2X or 4X
 *                         decoding. If 4X is selected, then an encoder FPGA
 *                         object is used and the returned counts will be 4x
 *                         the encoder spec'd value since all rising and
 *                         falling edges are counted. If 1X or 2X are selected
 *                         then a counter object will be used and the returned
 *                         value will either exactly match the spec'd count or
 *                         be double (2x) the spec'd count.
 */
Encoder::Encoder(int aChannel, int bChannel, bool reverseDirection,
                 EncodingType encodingType) {
  m_aSource = std::make_shared<DigitalInput>(aChannel);
  m_bSource = std::make_shared<DigitalInput>(bChannel);
  InitEncoder(reverseDirection, encodingType);
}

/**
 * Free the resources for an Encoder.
 *
 * Frees the FPGA resources associated with an Encoder.
 */
Encoder::~Encoder() {
  int32_t status = 0;
  HAL_FreeEncoder(m_encoder, &status);
  HAL_FATAL_WITH_STATUS(status);
}

/**
 * The encoding scale factor 1x, 2x, or 4x, per the requested encodingType.
 *
 * Used to divide raw edge counts down to spec'd counts.
 */
int Encoder::GetEncodingScale() const {
  int32_t status = 0;
  int val = HAL_GetEncoderEncodingScale(m_encoder, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return val;
}

/**
 * Gets the raw value from the encoder.
 *
 * The raw value is the actual count unscaled by the 1x, 2x, or 4x scale
 * factor.
 *
 * @return Current raw count from the encoder
 */
int Encoder::GetRaw() const {
  int32_t status = 0;
  int value = HAL_GetEncoderRaw(m_encoder, &status);
  HAL_FATAL_WITH_STATUS(status);
  return value;
}

/**
 * Returns the period of the most recent pulse.
 *
 * Returns the period of the most recent Encoder pulse in seconds.
 * This method compensates for the decoding type.
 *
 * @deprecated Use GetRate() in favor of this method.  This returns unscaled
 *             periods and GetRate() scales using value from
 *             SetDistancePerPulse().
 *
 * @return Period in seconds of the most recent pulse.
 */
double Encoder::GetPeriod() const {
  int32_t status = 0;
  double value = HAL_GetEncoderPeriod(m_encoder, &status);
  HAL_FATAL_WITH_STATUS(status);
  return value;
}
/**
 * Sets the maximum period for stopped detection.
 *
 * Sets the value that represents the maximum period of the Encoder before it
 * will assume that the attached device is stopped. This timeout allows users
 * to determine if the wheels or other shaft has stopped rotating.
 * This method compensates for the decoding type.
 *
 * @deprecated Use SetMinRate() in favor of this method.  This takes unscaled
 *             periods and SetMinRate() scales using value from
 *             SetDistancePerPulse().
 *
 * @param maxPeriod The maximum time between rising and falling edges before
 *                  the FPGA will report the device stopped. This is expressed
 *                  in seconds.
 */
void Encoder::SetMaxPeriod(double maxPeriod) {
  int32_t status = 0;
  HAL_SetEncoderMaxPeriod(m_encoder, maxPeriod, &status);
  HAL_FATAL_WITH_STATUS(status);
}

int Encoder::GetFPGAIndex() const {
  int32_t status = 0;
  int val = HAL_GetEncoderFPGAIndex(m_encoder, &status);
  HAL_FATAL_WITH_STATUS(status);
  return val;
}
