/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __READER_H__
#define __READER_H__

#include "NetworkTables/Entry.h"
#include <memory>
#include <string>

namespace NetworkTables
{
class Connection;

class Reader
{
public:
    Reader(Connection *connection, int inputStreamFd);
    int Read();
    std::string ReadString();
    int ReadId(bool useLastValue);
    int ReadTableId(bool useLastValue);
    int ReadInt();
    double ReadDouble();
    int ReadConfirmations(bool useLastValue);
    int ReadDenials(bool useLastValue);
    std::auto_ptr<Entry> ReadEntry(bool useLastValue);
private:
    int Check(bool useLastValue);
    int ReadVariableSize(bool useLastValue, int tag);

    /** The connection associated with this reader */
    Connection *m_connection;
    /** The input stream */
    int m_inputStreamFd;
    /** The last read value (-2 if nothing has been read) */
    int m_lastByte;

};

} // namespace

#endif
