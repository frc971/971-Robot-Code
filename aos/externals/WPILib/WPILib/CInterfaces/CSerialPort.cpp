/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#include "CInterfaces/CSerialPort.h"
#include <visa/visa.h>

static SerialPort* serial_port = NULL;

/**
 * Open the serial port object.
 * Open and allocate the serial port.
 *
 * @param baudRate The baud rate to configure the serial port.  The cRIO-9074 supports up to 230400 Baud.
 * @param dataBits The number of data bits per transfer.  Valid values are between 5 and 8 bits.
 * @param parity Select the type of parity checking to use.
 * @param stopBits The number of stop bits to use as defined by the enum StopBits.
 */
void OpenSerialPort(UINT32 baudRate, UINT8 dataBits, SerialPort::Parity parity, SerialPort::StopBits stopBits)
{
	if (serial_port == NULL)
	{
		serial_port = new SerialPort(baudRate, dataBits, parity, stopBits);
	}
}

/**
 * Set the type of flow control to enable on this port.
 *
 * By default, flow control is disabled.
 */
void SetSerialFlowControl(SerialPort::FlowControl flowControl)
{
	serial_port->SetFlowControl(flowControl);
}

/**
 * Enable termination and specify the termination character.
 *
 * Termination is currently only implemented for receive.
 * When the the terminator is recieved, the Read() or Scanf() will return
 *   fewer bytes than requested, stopping after the terminator.
 *
 * @param terminator The character to use for termination.
 */
void EnableSerialTermination(char terminator)
{
	serial_port->EnableTermination(terminator);
}

/**
 * Disable termination behavior.
 */
void DisableSerialTermination()
{
	serial_port->DisableTermination();
}

/**
 * Get the number of bytes currently available to read from the serial port.
 *
 * @return The number of bytes available to read.
 */
INT32 GetSerialBytesReceived()
{
	return serial_port->GetBytesReceived();
}

/**
 * Output formatted text to the serial port.
 *
 * @bug All pointer-based parameters seem to return an error.
 *
 * @param writeFmt A string that defines the format of the output.
 */
void PrintfSerial(const char *writeFmt, ...)
{
	va_list args;

	va_start (args, writeFmt);
	serial_port->Printf((ViString)writeFmt, args);
	va_end (args);
}

/**
 * Input formatted text from the serial port.
 *
 * @bug All pointer-based parameters seem to return an error.
 *
 * @param readFmt A string that defines the format of the input.
 */
void ScanfSerial(const char *readFmt, ...)
{
	va_list args;

	va_start (args, readFmt);
	serial_port->Scanf((ViString)readFmt, args);
	va_end (args);
}

/**
 * Read raw bytes out of the buffer.
 *
 * @param buffer Pointer to the buffer to store the bytes in.
 * @param count The maximum number of bytes to read.
 * @return The number of bytes actually read into the buffer.
 */
UINT32 ReadSerialPort(char *buffer, INT32 count)
{
	return serial_port->Read(buffer, count);
}

/**
 * Write raw bytes to the buffer.
 *
 * @param buffer Pointer to the buffer to read the bytes from.
 * @param count The maximum number of bytes to write.
 * @return The number of bytes actually written into the port.
 */
UINT32 WriteSerialPort(const char *buffer, INT32 count)
{
	return serial_port->Write(buffer, count);
}

/**
 * Configure the timeout of the serial port.
 *
 * This defines the timeout for transactions with the hardware.
 * It will affect reads and very large writes.
 *
 * @param timeout The number of seconds to to wait for I/O.
 */
void SetSerialTimeout(INT32 timeout)
{
	serial_port->SetTimeout(timeout);
}

/**
 * Specify the flushing behavior of the output buffer.
 *
 * When set to kFlushOnAccess, data is synchronously written to the serial port
 *   after each call to either Printf() or Write().
 *
 * When set to kFlushWhenFull, data will only be written to the serial port when
 *   the buffer is full or when Flush() is called.
 *
 * @param mode The write buffer mode.
 */
void SetSerialWriteBufferMode(SerialPort::WriteBufferMode mode)
{
	serial_port->SetWriteBufferMode(mode);
}

/**
 * Force the output buffer to be written to the port.
 *
 * This is used when SetWriteBufferMode() is set to kFlushWhenFull to force a
 * flush before the buffer is full.
 */
void FlushSerialPort()
{
	serial_port->Flush();
}

/**
 * Reset the serial port driver to a known state.
 *
 * Empty the transmit and receive buffers in the device and formatted I/O.
 */
void ResetSerialPort()
{
	serial_port->Reset();
}
