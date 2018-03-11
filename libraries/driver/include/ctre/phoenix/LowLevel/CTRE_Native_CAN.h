#pragma once

#include <FRC_NetworkCommunication/CANSessionMux.h>  //CAN Comm

int CTRE_Native_CAN_GetSendBuffer(int arbId, uint64_t & data);
int CTRE_Native_CAN_Receive(int arbId, uint64_t & data, int & len, bool allowStale = true);
int CTRE_Native_CAN_Send(int arbId, uint64_t data, int len, int periodMs);
