#ifndef AOS_ATOM_CODE_OUTPUT_MOTOR_OUTPUT_H_
#define AOS_ATOM_CODE_OUTPUT_MOTOR_OUTPUT_H_

#include <stdint.h>
#include <string.h>
#include <algorithm>
#include <string>

#include "aos/common/network/SendSocket.h"
#include "aos/common/byteorder.h"
#include "aos/common/type_traits.h"

namespace aos {

class MotorOutput {
 public:
  MotorOutput();
  void Run();

  // Constants for the first argument of AddMotor.
  static const char VICTOR = 'v';
  static const char TALON = 't';

 protected:
  // num is 1-indexed
  int AddMotor(char type, uint8_t num, float value);
  int AddSolenoid(uint8_t num, bool on);
  int AddDSLine(uint8_t line, const char *data);
  // loop_queuegroup is expected to be a control loop queue group.
  // This function will fetch the latest goal and serve it.
  template <class T>
  int AddControlLoopGoal(T *loop_queuegroup);

  virtual void RunIteration() = 0;

 private:
  SendSocket sock;
};

template <class T>
int MotorOutput::AddControlLoopGoal(T *loop_queuegroup) {
  typedef typename std::remove_reference<
      decltype(*(loop_queuegroup->goal.MakeMessage().get()))>::type GoalType;
  // TODO(aschuh): This assumes a static serialized message size.
  const uint16_t size = GoalType::Size();
  if (sock.hold_msg_len_ + 4 + 1 + size > sock.MAX_MSG) {
    return -1;
  }

  const bool zero = !loop_queuegroup->goal.FetchLatest();

  sock.hold_msg_[sock.hold_msg_len_ ++] = 'g';
  const uint32_t hash = loop_queuegroup->hash();

  // Place the loop hash at the front.
  to_network(&hash, &sock.hold_msg_[sock.hold_msg_len_]);
  sock.hold_msg_len_ += 4;

  if (zero) {
    GoalType zero_message;
    zero_message.Zero();
    zero_message.Serialize(&sock.hold_msg_[sock.hold_msg_len_]);
    sock.hold_msg_len_ += size;
    return -1;
  } else {
    loop_queuegroup->goal->Serialize(&sock.hold_msg_[sock.hold_msg_len_]);
    sock.hold_msg_len_ += size;
    return 0;
  }
}

}  // namespace aos

#endif
