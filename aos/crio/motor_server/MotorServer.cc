#include "aos/crio/motor_server/MotorServer.h"

#include <stdio.h>
#include <stdlib.h>
#include "usrLib.h"

#include "WPILib/Timer.h"
#include "WPILib/Task.h"

#include "aos/common/inttypes.h"
#include "aos/crio/motor_server/MotorControllerOutput.h"
#include "aos/crio/motor_server/SolenoidOutput.h"
#include "aos/common/Configuration.h"

namespace aos {
namespace crio {

ByteBuffer MotorServer::buff(4096);
DriverStationLCD *MotorServer::ds_lcd(NULL);
int MotorServer::count(0);
ReceiveSocket *MotorServer::sock(NULL);
SEM_ID MotorServer::motorSync = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
OutputDevice *MotorServer::output_devices[256][kMaxOutputDeviceNumber];
Mutex MotorServer::control_loop_goals_lock;
::std::map<uint32_t,
           control_loops::SerializableControlLoop *> MotorServer::loops;
Task *MotorServer::tcpTask;
void MotorServer::Start() {
  sock = new ReceiveSocket(NetworkPort::kMotors);

  memset(output_devices, 0x00, sizeof(output_devices));

  tcpTask = new Task("MRLoop",
                     reinterpret_cast<FUNCPTR>(RunReaderTask),
                     WORK_PRIORITY);

  tcpTask->Start();
}

void MotorServer::ProcessBuf() {
  semTake(motorSync, WAIT_FOREVER);
  bool cont = true;
  while (true) {
    if (!cont) {
      LOG(WARNING, "Malformed Packet\n");
      goto end;
    }
    switch (const int value = buff.read_char()) {
      case 'g':
        cont = ProcessControlLoopGoal();
        break;
      case 'd':
        cont = ProcessDSLine();
        break;
      case -1:
        goto end;
      default:
        cont = ProcessOutputDevice(value);
        break;
    }
  }
end:
  ++count;
  semGive(motorSync);
}
bool MotorServer::ProcessOutputDevice(const int type) {
  const int id = buff.read_char(); // 1-indexed
  if (id < 1 || id > static_cast<ssize_t>(kMaxOutputDeviceNumber)) {
    if (id != -1) {
      LOG(ERROR, "illegal OutputDevice id %d\n", id);
    }
    return false;
  }

  if (output_devices[type][id - 1] == NULL) {
    switch (type) {
      case 'v':
        output_devices[type][id - 1] = new VictorOutput(id);
        break;
      case 'j':
        output_devices[type][id - 1] = new JaguarOutput(id);
        break;
      case 'c':
        output_devices[type][id - 1] = new CANJaguarOutput(id);
        break;
      case 't':
        output_devices[type][id - 1] = new TalonOutput(id);
        break;
      case 's':
        output_devices[type][id - 1] = new SolenoidOutput(id);
        break;
      default:
        LOG(ERROR, "unrecognized OutputDevice type %d\n", type);
        return false;
    }
  }
  return output_devices[type][id - 1]->ReadValue(buff);
}

bool MotorServer::ProcessDSLine() {
  int line = buff.read_char();
  if (line == -1) {
    return false;
  }
  // TODO(brians): Subfunction
  DriverStationLCD::Line ds_line;
  switch (line) {
    case 0:
      ds_line = DriverStationLCD::kMain_Line6;
      break;
    case 1:
      ds_line = DriverStationLCD::kUser_Line1;
      break;
    case 2:
      ds_line = DriverStationLCD::kUser_Line2;
      break;
    case 3:
      ds_line = DriverStationLCD::kUser_Line3;
      break;
    case 4:
      ds_line = DriverStationLCD::kUser_Line4;
      break;
    case 5:
      ds_line = DriverStationLCD::kUser_Line5;
      break;
    case 6:
      ds_line = DriverStationLCD::kUser_Line6;
      break;
    default:
      LOG(ERROR, "illegal line number %hhd\n", line);
      return false;
  }
  // TODO(brians) see if this mess with not creating the DriverStationLCD for a
  // bit is actually needed
  static int ds_lcd_counts = 0; // to prevent crashes at startup
  if (ds_lcd == NULL) {
    if (ds_lcd_counts < 100) {
      ++ds_lcd_counts;
    } else {
      ++ds_lcd_counts;
      ds_lcd = DriverStationLCD::GetInstance();
    }
  }
  char buf[DriverStationLCD::kLineLength];
  buff.read_string(buf, sizeof(buf));
  buf[sizeof(buf) - 1] = 0;
  if (ds_lcd != NULL) {
    ds_lcd->PrintfLine(ds_line, "%s", buf);
  }
  return true;
}

void MotorServer::RegisterControlLoopGoal(
    control_loops::SerializableControlLoop *control_loop) {
  uint32_t unique_id = control_loop->UniqueID();

  bool replaced;
  {
    MutexLocker control_loop_goals_locker(&control_loop_goals_lock);
    replaced = !InsertIntoMap(&loops, unique_id, control_loop);
  }
  if (replaced) {
    LOG(ERROR, "Replaced a key for unique id 0x%"PRIx32"\n", unique_id);
  }
}

bool MotorServer::ProcessControlLoopGoal() {
  // Read back a uint32_t with the hash.
  uint32_t hash;
  if (!buff.read_uint32(&hash)) return false;
  MutexLocker control_loop_goals_locker(&control_loop_goals_lock);

  control_loops::SerializableControlLoop *loop;
  if (!GetFromMap(loops, hash, &loop)) {
    return false;
  }
  const size_t length = loop->SeralizedSize();
  char *const goal_bytes = buff.get_bytes(length);
  if (goal_bytes == NULL) {
    return false;
  } else {
    loop->Deserialize(goal_bytes);
  }
  return true;
}

void MotorServer::RunReaderTask() {
  while (true) {
    if (buff.recv_from_sock(sock)) {
      ProcessBuf();
    }
  }
}
void MotorServer::WriteOutputs() {
  static int last_count = 0, bad_counts = 0;
  semTake(motorSync, WAIT_FOREVER);
  if (last_count != count) {
    bad_counts = 0;
  } else {
    ++bad_counts;
  }
  last_count = count;
  // both loops iterate over all elements of output_devices by indexing off the
  // end of output_devices[0]
  if (bad_counts > 2) {
    LOG(WARNING, "no new values. stopping all outputs\n");
    for (size_t i = 0; i < sizeof(output_devices) / sizeof(output_devices[0][0]); ++i) {
      if (output_devices[0][i] != NULL) {
        output_devices[0][i]->NoValue();
      }
    }
  } else {
    for (size_t i = 0; i < sizeof(output_devices) / sizeof(output_devices[0][0]); ++i) {
      if (output_devices[0][i] != NULL) {
        output_devices[0][i]->SetValue();
      }
    }
  }
  if (ds_lcd != NULL) {
    ds_lcd->UpdateLCD();
  }
  semGive(motorSync);
}

}  // namespace crio
}  // namespace aos
