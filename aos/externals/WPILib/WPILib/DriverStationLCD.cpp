/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "DriverStationLCD.h"
#include "NetworkCommunication/FRCComm.h"
#include "NetworkCommunication/UsageReporting.h"
#include "Synchronized.h"
#include "WPIErrors.h"
#include <strLib.h>

const UINT32 DriverStationLCD::kSyncTimeout_ms;
const UINT16 DriverStationLCD::kFullDisplayTextCommand;
const INT32 DriverStationLCD::kLineLength;
const INT32 DriverStationLCD::kNumLines;
DriverStationLCD* DriverStationLCD::m_instance = NULL;

/**
 * DriverStationLCD contructor.
 * 
 * This is only called once the first time GetInstance() is called
 */
DriverStationLCD::DriverStationLCD()
	: m_textBuffer (NULL)
	, m_textBufferSemaphore (NULL)
{
	m_textBuffer = new char[USER_DS_LCD_DATA_SIZE];
	memset(m_textBuffer, ' ', USER_DS_LCD_DATA_SIZE);

	*((UINT16 *)m_textBuffer) = kFullDisplayTextCommand;

	m_textBufferSemaphore = semMCreate(SEM_DELETE_SAFE);

	nUsageReporting::report(nUsageReporting::kResourceType_DriverStationLCD, 0);

	AddToSingletonList();
}

DriverStationLCD::~DriverStationLCD()
{
	semDelete(m_textBufferSemaphore);
	delete [] m_textBuffer;
	m_instance = NULL;
}

/**
 * Return a pointer to the singleton DriverStationLCD.
 */
DriverStationLCD* DriverStationLCD::GetInstance()
{
	if (m_instance == NULL)
	{
		m_instance = new DriverStationLCD();
	}
	return m_instance;
}

/**
 * Send the text data to the Driver Station.
 */
void DriverStationLCD::UpdateLCD()
{
	Synchronized sync(m_textBufferSemaphore);
	setUserDsLcdData(m_textBuffer, USER_DS_LCD_DATA_SIZE, kSyncTimeout_ms);
}

/**
 * Print formatted text to the Driver Station LCD text bufer.
 * 
 * Use UpdateLCD() periodically to actually send the text to the Driver Station.
 * 
 * @param line The line on the LCD to print to.
 * @param startingColumn The column to start printing to.  This is a 1-based number.
 * @param writeFmt The printf format string describing how to print.
 */
void DriverStationLCD::Printf(Line line, INT32 startingColumn, const char *writeFmt, ...)
{
	va_list args;
	va_start (args, writeFmt);
	VPrintf(line, startingColumn, writeFmt, args);
	va_end (args);
}

void DriverStationLCD::VPrintf(Line line, INT32 startingColumn, const char *writeFmt, va_list args)
{
	UINT32 start = startingColumn - 1;
	INT32 maxLength = kLineLength - start;
	char lineBuffer[kLineLength + 1];

	if (startingColumn < 1 || startingColumn > kLineLength)
	{
		wpi_setWPIErrorWithContext(ParameterOutOfRange, "startingColumn");
		return;
	}

	if (line < kMain_Line6 || line > kUser_Line6)
	{
		wpi_setWPIErrorWithContext(ParameterOutOfRange, "line");
		return;
	}

	{
		Synchronized sync(m_textBufferSemaphore);
		// snprintf appends NULL to its output.  Therefore we can't write directly to the buffer.
		INT32 length = vsnprintf(lineBuffer, kLineLength + 1, writeFmt, args);
		if (length < 0) length = kLineLength;

		memcpy(m_textBuffer + start + line * kLineLength + sizeof(UINT16), lineBuffer, std::min(maxLength,length));
	}
}

/**
 * Print formatted text to the Driver Station LCD text bufer. This function 
 * pads the line with empty spaces. 
 * 
 * Use UpdateLCD() periodically to actually send the text to the Driver Station.
 * 
 * @param line The line on the LCD to print to.
 * @param writeFmt The printf format string describing how to print.
 */
void DriverStationLCD::PrintfLine(Line line, const char *writeFmt, ...)
{
	va_list args;
	va_start (args, writeFmt);
	VPrintfLine(line, writeFmt, args);
	va_end (args);
}

void DriverStationLCD::VPrintfLine(Line line, const char *writeFmt, va_list args)
{
	char lineBuffer[kLineLength + 1];

	if (line < kMain_Line6 || line > kUser_Line6)
	{
		wpi_setWPIErrorWithContext(ParameterOutOfRange, "line");
		return;
	}

	{
		Synchronized sync(m_textBufferSemaphore);
		// snprintf appends NULL to its output.  Therefore we can't write directly to the buffer.
		INT32 length = std::min(vsnprintf(lineBuffer, kLineLength + 1, writeFmt, args), kLineLength);
		if (length < 0) length = kLineLength;

		// Fill the rest of the buffer
		if (length < kLineLength)
		{
			memset(lineBuffer + length, ' ', kLineLength - length);
		}
		
		memcpy(m_textBuffer + line * kLineLength + sizeof(UINT16), lineBuffer, kLineLength);
	}
}

/**
 *  Clear all lines on the LCD.
 */
void DriverStationLCD::Clear()
{
	Synchronized sync(m_textBufferSemaphore);
	memset(m_textBuffer + sizeof(UINT16), ' ', kLineLength*kNumLines);
}

