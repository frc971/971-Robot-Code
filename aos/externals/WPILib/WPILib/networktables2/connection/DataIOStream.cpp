#include "networktables2/connection/DataIOStream.h"

//TODO remove this when alloca is solved
#ifdef WIN32
#include <malloc.h>
#endif

DataIOStream::DataIOStream(IOStream* _iostream) :
	iostream(*_iostream)
{
}
DataIOStream::~DataIOStream()
{
	close();
	delete &iostream;
}
void DataIOStream::close()
{
	iostream.close();
}

void DataIOStream::writeByte(uint8_t b)
{
	iostream.write(&b, 1);
}
void DataIOStream::write2BytesBE(uint16_t s)
{
	writeByte((uint8_t)(s >> 8));
	writeByte((uint8_t)s);
}
void DataIOStream::writeString(std::string& str)
{
	write2BytesBE(str.length());
	iostream.write(str.c_str(), str.length());
}
void DataIOStream::flush()
{
	iostream.flush();
}
uint8_t DataIOStream::readByte()
{
	uint8_t value;
	iostream.read(&value, 1);
	return value;
}
uint16_t DataIOStream::read2BytesBE()
{
	uint16_t value;
	value = readByte()<<8 | readByte();
	return value;
}
std::string* DataIOStream::readString()
{
	
	unsigned int byteLength = read2BytesBE();
#ifndef WIN32
	uint8_t bytes[byteLength+1];//FIXME figure out why this doesn't work on windows
#else
	uint8_t* bytes = (uint8_t*)alloca(byteLength+1);
#endif
	iostream.read(bytes, byteLength);
	bytes[byteLength] = 0;
	return new std::string((char*)bytes);//FIXME implement UTF-8 aware version
}
