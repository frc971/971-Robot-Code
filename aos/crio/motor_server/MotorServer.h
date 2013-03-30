#ifndef AOS_CRIO_MOTOR_SERVER_MOTOR_SERVER_H_
#define AOS_CRIO_MOTOR_SERVER_MOTOR_SERVER_H_

#include <vxWorks.h>
#include <timers.h>
#include <string.h>
#include "WPILib/Task.h"
#include "WPILib/Victor.h"
#include "WPILib/Jaguar.h"
#include "WPILib/Solenoid.h"
#include "sockLib.h"
#include <inetLib.h>
#include <stdio.h>
#include <selectLib.h>
#include <stdlib.h>
#include <time.h>
#include <map>
#include <string>

#include "WPILib/DriverStationLCD.h"

#include "aos/common/control_loop/ControlLoop.h"
#include "aos/common/inttypes.h"
#include "aos/common/messages/QueueHolder.h"
#include "aos/common/mutex.h"
#include "aos/common/network/ReceiveSocket.h"
#include "aos/common/network/SendSocket.h"
#include "aos/crio/motor_server/ControlLoopGoals.h"
#include "aos/crio/motor_server/OutputDevice.h"
#include "aos/crio/shared_libs/ByteBuffer.h"
#include "aos/map_utils.h"

namespace aos {
namespace crio {

template<class Values>
class CRIOControlLoopRunner;
class MotorServer {
 public:
  static void Start();

  // Adds the given control loop's goal queue to the list of ones to process.
  static void RegisterControlLoopGoal(
      control_loops::SerializableControlLoop *control_loop);

  static const int32_t WORK_PRIORITY = 100;

  // Needs to be called by some other piece of code every 10ms (after any
  // control loops have been run etc).
  static void WriteOutputs();

 private:
  template<class Values>
  friend class CRIOControlLoopRunner;

  // Counter for how many times new values come in. Used to stop all the
  // outputs if values stop.
  // Would take days to overflow.
  static int count;
  static SEM_ID motorSync;

  static void RunReaderTask();
  static Task *tcpTask;
  static ReceiveSocket *sock;
  static ByteBuffer buff;

  static DriverStationLCD *ds_lcd;
  static bool ProcessDSLine();

  static const size_t kMaxOutputDeviceNumber = 10;
  static OutputDevice *output_devices[256][kMaxOutputDeviceNumber];
  static bool ProcessOutputDevice(const int type);

  // Go through the whole buffer and call the appropriate Process* methods to
  // process each part.
  static void ProcessBuf();

  static bool ProcessControlLoopGoal();
  // Locked whenever adding/using the control loop goals maps.
  // Also used by CRIOControlLoopRunner while modifying any of the data
  // structures.  Used by both of them while reading/writing from
  // the goal queues.
  static Mutex control_loop_goals_lock;
  static ::std::map<uint32_t, control_loops::SerializableControlLoop *> loops;
};

}  // namespace crio
}  // namespace aos

#endif
