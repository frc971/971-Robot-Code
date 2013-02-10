/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __DATA_H__
#define __DATA_H__

namespace NetworkTables
{

class Buffer;

class Data {
public:
	virtual void Encode(Buffer *buffer) = 0;
	virtual bool IsEntry() {return false;}
	virtual bool IsOldData() {return false;}
	virtual bool IsTransaction() {return false;}
};

} // namespace

#endif
