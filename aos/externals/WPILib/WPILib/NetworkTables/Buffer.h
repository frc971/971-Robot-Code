/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __BUFFER_H__
#define __BUFFER_H__

#include "ErrorBase.h"
#include <vxWorks.h>
#include <string>

namespace NetworkTables
{

class Buffer : public ErrorBase
{
public:
	Buffer(UINT32 capacity);
	~Buffer();
	void WriteString(UINT32 length, const char *entry);
	void WriteString(const char *entry);
	void WriteString(std::string entry);
	void WriteDouble(double entry);
	void WriteInt(UINT32 entry);
	void WriteId(UINT32 id);
	void WriteTableId(UINT32 id);
	void WriteBytes(UINT32 length, const UINT8 *entries);
	void WriteByte(UINT8 entry);
	void Flush(int socket);
	void Clear();

private:
	void WriteVariableSize(UINT32 tag, UINT32 id);

	UINT8 *m_buffer;
	UINT32 m_size;
	UINT32 m_capacity;
};

} // namespace

#endif

