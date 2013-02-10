/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "NetworkTables/Buffer.h"
#include "NetworkTables/InterfaceConstants.h"
#include "WPIErrors.h"
#include <sockLib.h>
#include <string.h>

namespace NetworkTables
{

Buffer::Buffer(UINT32 capacity) :
	m_buffer (NULL),
	m_size (0),
	m_capacity (capacity)
{
	m_buffer = new UINT8[capacity];
}

Buffer::~Buffer()
{
	delete [] m_buffer;
}

void Buffer::WriteString(UINT32 length, const char *entry)
{
	if (length >= kNetworkTables_BEGIN_STRING)
	{
		WriteByte(kNetworkTables_BEGIN_STRING);
		WriteBytes(length, (UINT8*)entry);
		WriteByte(kNetworkTables_END_STRING);
	}
	else
	{
		WriteByte(length);
		WriteBytes(length, (UINT8*)entry);
	}
}

void Buffer::WriteString(const char *entry)
{
	WriteString(strlen(entry), entry);
}

void Buffer::WriteString(std::string entry)
{
	WriteString(entry.length(), entry.c_str());
}

void Buffer::WriteDouble(double entry)
{
	WriteBytes(sizeof(entry), (UINT8*) &entry);
}

void Buffer::WriteInt(UINT32 entry)
{
	WriteBytes(sizeof(entry), (UINT8*) &entry);
}

void Buffer::WriteId(UINT32 id)
{
	WriteVariableSize(kNetworkTables_ID, id);
}

void Buffer::WriteTableId(UINT32 id)
{
	WriteVariableSize(kNetworkTables_TABLE_ID, id);
}

void Buffer::WriteBytes(UINT32 length, const UINT8 *entries)
{
	UINT32 i;
	for (i = 0; i < length; i++)
	{
		WriteByte(entries[i]);
	}
}

void Buffer::WriteByte(UINT8 entry)
{
	if (m_size >= m_capacity)
	{
		wpi_setWPIError(NetworkTablesBufferFull);
		return;
	}
	m_buffer[m_size++] = entry;
}

void Buffer::Flush(int socket)
{
#ifdef DEBUG
	if (m_size != 1 || m_buffer[0] != kNetworkTables_PING)
	{
		unsigned i;
		char buf[128];
		char *pbuf = buf;
		pbuf = pbuf + snprintf(pbuf, 128 - (pbuf - buf), "\nO:");
		for (i=0; i<m_size; i++)
		{
			pbuf = pbuf + snprintf(pbuf, 128 - (pbuf - buf), "%02X", m_buffer[i]);
		}
		snprintf(pbuf, 128 - (pbuf - buf), "\n");
		printf(buf);
	}
#endif
	write(socket, (char *)m_buffer, m_size);
	Clear();
}

void Buffer::Clear()
{
	m_size = 0;
}

void Buffer::WriteVariableSize(UINT32 tag, UINT32 id)
{
	if (id < tag - 4)
	{
		WriteByte(tag | id);
	}
	else
	{
		int fullTag = (tag | (tag - 1)) ^ 3;
		if (id < (1 << 8))
		{
			WriteByte(fullTag);
			WriteByte(id);
		}
		else if (id < (1 << 16))
		{
			WriteByte(fullTag | 1);
			WriteByte(id >> 8);
			WriteByte(id);
		}
		else if (id < (1 << 24))
		{
			WriteByte(fullTag | 2);
			WriteByte(id >> 16);
			WriteByte(id >> 8);
			WriteByte(id);
		}
		else
		{
			WriteByte(fullTag | 3);
			WriteByte(id >> 24);
			WriteByte(id >> 16);
			WriteByte(id >> 8);
			WriteByte(id);
		}
	}
}

} // namespace
