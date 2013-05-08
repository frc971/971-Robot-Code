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
#include "WPILib/Task.h"

// A simple implementation of receiving motor values over UDP and sending
// joystick data back out.
// Deals with turning a compressor on and off at the same time.
// You should not try to change any of the outputs on any of the modules that
// you have this class control outside of this class.
// This class takes care of disabling outputs when no packets are received in
// kDisableTime and feeding the Watchdog correctly.
//
// The receiving interface consists of receiving NetworkRobotValues structs on
// a given UDP port from a given sender address. Each time a set of motor values
// is received, this class will update all output values.
//
// The sending interface consists of sending NetworkRobotJoysticks structs on a
// given UDP port to a given sender address. Each time a new Driver's Station
// packet is received from the FMS code this class will send out another packet
// with the new values.
class NetworkRobot : public RobotBase, public ErrorBase {
 protected:
  // Does not take ownership of *sender_address or *receiver_address.
  // A NULL for either address means to not do anything with that part (sending
  // or receiving).
  NetworkRobot(UINT16 receive_port, const char *sender_address,
               UINT16 send_port, const char *receiver_address);
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
  // Must stay less than 1 second with the current implementation.
  static const double kDisableTime;

  // Deals with calling inet_aton and passing any errors on to the logging
  // system. A helper is necessary here because inet_aton is normally just kind
  // of annoying to deal with, but under vxworks, you have to make a copy of the
  // string before doing anything with etc etc so it's really complicated.
  // Returns whether or not it was successful. If it returns false, then an
  // error will have already been recorded.
  bool FillinInAddr(const char *const_ip, in_addr *inet_address);

  // Cleans everything up after the main loop encounters an error so that it can
  // try again.
  void Cleanup();

  // Waits for receive_socket_ to become readable for up to kDisableTime.
  // Returns whether or not there is readable data available.
  bool WaitForData();

  // Receives a packet and calls ProcessPacket() if it's a good one.
  void ReceivePacket();

  // Gets run in its own task to take DS data and send it out.
  void SendLoop();
  static void StaticSendLoop(void *self) {
    static_cast<NetworkRobot *>(self)->SendLoop();
  }

  // Sets receive_socket_ to an opened socket listening on receive_port_ to UDP
  // packets from sender_address_.
  void CreateReceiveSocket();
  // Sets send_socket_ to an opened socket sending UDP packets on send_port_ to
  // receiver_address_.
  void CreateSendSocket();

  const UINT16 receive_port_;
  const char *const sender_address_;
  struct in_addr expected_sender_address_;

  const UINT16 send_port_;
  const char *const receiver_address_;

  int receive_socket_;
  NetworkRobotMotors values_;

  int send_socket_;
  NetworkRobotJoysticks joystick_values_;
  Task send_task_;

  // Helper function to copy all of the data for a single joystick into
  // joystick_values_.
  // axes and buttons get copied into joystick_values_.joysticks[number].
  void CopyStickValues(int number, const INT8 (&axes)[6], UINT16 buttons);

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
