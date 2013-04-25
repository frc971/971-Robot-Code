/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_H_
#define WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_H_

#include <inetLib.h>

#include "WPILib/NetworkRobot/NetworkRobotValues.h"
#include "WPILib/RobotBase.h"
#include "WPILib/Base.h"
#include "WPILib/DigitalModule.h"
#include "WPILib/ErrorBase.h"
#include "WPILib/SolenoidBase.h"

// A simple implementation of receiving motor values over UDP.
// Deals with turning a compressor on and off at the same time.
// You should not try to change any of the outputs on any of the modules that
// you have this class control outside of this class.
// This class takes care of disabling outputs when no packets are received in
// kDisableTime and feeding the Watchdog correctly.
//
// The interface consists of receiving NetworkRobotValues structs on a given UDP
// port from a given sender address. Each time a set of motor values is
// received, this class will update all output values.
class NetworkRobot : public RobotBase, public ErrorBase {
 protected:
  // Does not take ownership of *sender_address.
  NetworkRobot(UINT16 port, const char *sender_address);
  virtual ~NetworkRobot();

  virtual void StartCompetition();

  // Called when a valid packet has been received into values_.
  // Subclasses can override if they want to do more, however they still need to
  // call this implementation.
  virtual void ProcessPacket();

  // Called when no packet has been received for too long and it's time to stop
  // everything.
  // Subclasses can override if they want to do more, however they still need to
  // call this implementation.
  virtual void StopOutputs();

 private:
  // How long between packets we wait until we disable all of the outputs (in
  // seconds).
  // Must stay less than 1 second.
  static const double kDisableTime;

  // Waits for socket_ to become readable for up to kDisableTime.
  // Returns whether or not there is readable data available.
  bool WaitForData();

  // Receives a packet and calls ProcessPacket() if it's a good one.
  void ReceivePacket();

  // Sets socket_ to an opened socket listening on port to UDP packets from
  // sender_address.
  void CreateSocket();

  const UINT16 port_;
  const char *const sender_address_;
  struct in_addr expected_sender_address_;

  NetworkRobotValues values_;
  int socket_;

  // Using Timer::GetPPCTimestamp() values.
  double last_received_timestamp_;

  // Ownership of the pointers stored here stays in DigitalModule.
  DigitalModule *digital_modules_[2];
  // This object owns pointers stored here.
  SolenoidBase *solenoid_bases_[2];

  // A bitmask of all of the digital outputs that we have currently allocated.
  // In hardware order.
  UINT16 allocated_digital_outputs_[2];

  DISALLOW_COPY_AND_ASSIGN(NetworkRobot);
};

#endif  // WPILIB_NETWORK_ROBOT_NETWORK_ROBOT_H_
