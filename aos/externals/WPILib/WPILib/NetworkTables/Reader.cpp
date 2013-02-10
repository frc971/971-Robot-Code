/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/Reader.h"

#include "NetworkTables/BooleanEntry.h"
#include "NetworkTables/Connection.h"
#include "NetworkTables/DoubleEntry.h"
#include "NetworkTables/IntegerEntry.h"
#include "NetworkTables/StringEntry.h"
#include "WPIErrors.h"
#include <errnoLib.h> 
#include <selectLib.h> 
#include <sockLib.h> 
#include <usrLib.h> 

namespace NetworkTables
{

Reader::Reader(Connection *connection, int inputStreamFd) :
	m_connection(connection),
	m_inputStreamFd(inputStreamFd)
{
	m_lastByte = -2;
}

int Reader::Read()
{
	fd_set readFdSet;
	int retval = ERROR;

	FD_ZERO(&readFdSet);
	FD_SET(m_inputStreamFd, &readFdSet);
	if (select(FD_SETSIZE, &readFdSet, NULL, NULL, NULL) != ERROR)
	{
		if (FD_ISSET(m_inputStreamFd, &readFdSet))
		{
			char readbuf;
			retval = recv(m_inputStreamFd, &readbuf, 1, 0);
			m_lastByte = readbuf;
			if (retval != ERROR && retval > 0)
			{
#ifdef DEBUG
				if (m_lastByte != kNetworkTables_PING)
				{
					char pbuf[6];
					snprintf(pbuf, 6, "I:%02X\n", m_lastByte);
					printf(pbuf);
				}
#endif
				return m_lastByte;
			}
		}
	}
	else if (!m_connection->IsConnected())
	{
		// The error came from us closing the socket
		return 0;
	}

	// TODO: Should we ignore ECONNRESET errors?
	if (retval == ERROR)
	{
		char buf[32] = "";
		int err = errnoGet();
		snprintf(buf, 32, "errno=%d", err);
		wpi_setStaticWPIErrorWithContext(m_connection, NetworkTablesReadError, buf);
		printErrno(err);
	}
	m_connection->Close();
	return 0;
}

int Reader::Check(bool useLastValue)
{
	return useLastValue ? m_lastByte : Read();
}

std::string Reader::ReadString()
{
	Read();
	std::string buffer;

	if (m_lastByte == kNetworkTables_BEGIN_STRING)
	{
		buffer.reserve(360);
		buffer.clear();
		while (Read() != kNetworkTables_END_STRING)
			buffer.append(1, (char)m_lastByte);
	}
	else
	{
		int length = m_lastByte;
		buffer.reserve(length + 1);
		buffer.clear();
		for (int i = 0; i < length; i++)
			buffer[i] = (char)Read();
		buffer[length] = '\0';
	}

	return buffer;
}

int Reader::ReadId(bool useLastValue)
{
	return ReadVariableSize(useLastValue, kNetworkTables_ID);
}

int Reader::ReadTableId(bool useLastValue) {
	return ReadVariableSize(useLastValue, kNetworkTables_TABLE_ID);
}

int Reader::ReadVariableSize(bool useLastValue, int tag) {
	int value = Check(useLastValue);
	value ^= tag;
	if (value < tag - 4)
	{
		return value;
	}
	else
	{
		int bytes = (value & 3) + 1;
		int id = 0;
		for (int i = 0; i < bytes; i++)
			id = (id << 8) | Read();
		return id;
	}
}

int Reader::ReadInt()
{
	return (Read() << 24) | (Read() << 16) | (Read() << 8) | Read();
}

double Reader::ReadDouble()
{
	long l = Read();
	l = (l << 8) | Read();
	l = (l << 8) | Read();
	l = (l << 8) | Read();
	l = (l << 8) | Read();
	l = (l << 8) | Read();
	l = (l << 8) | Read();
	l = (l << 8) | Read();
	return *((double *)&l);
}

int Reader::ReadConfirmations(bool useLastValue)
{
	return Check(useLastValue) ^ kNetworkTables_CONFIRMATION;
}

int Reader::ReadDenials(bool useLastValue)
{
	return Check(useLastValue) ^ kNetworkTables_DENIAL;
}

std::auto_ptr<Entry> Reader::ReadEntry(bool useLastValue)
{
	switch (Check(useLastValue))
	{
	case kNetworkTables_BOOLEAN_FALSE:
		return std::auto_ptr<Entry>(new BooleanEntry(false));
	case kNetworkTables_BOOLEAN_TRUE:
		return std::auto_ptr<Entry>(new BooleanEntry(true));
	case kNetworkTables_INT:
		return std::auto_ptr<Entry>(new IntegerEntry(ReadInt()));
	case kNetworkTables_DOUBLE:
		return std::auto_ptr<Entry>(new DoubleEntry(ReadDouble()));
	case kNetworkTables_STRING:
		return std::auto_ptr<Entry>(new StringEntry(ReadString().c_str()));
	default:
		return std::auto_ptr<Entry>(NULL);
	}
}

} // namespace
