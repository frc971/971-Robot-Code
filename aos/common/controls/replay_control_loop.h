#ifndef AOS_COMMON_CONTROLS_REPLAY_CONTROL_LOOP_H_
#define AOS_COMMON_CONTROLS_REPLAY_CONTROL_LOOP_H_

#include <fcntl.h>

#include "aos/common/queue.h"
#include "aos/common/controls/control_loop.h"
#include "aos/common/logging/replay.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/time.h"
#include "aos/common/macros.h"

namespace aos {
namespace controls {

// Handles reading logged messages and running them through a control loop
// again.
// T should be a queue group suitable for use with ControlLoop.
template <class T>
class ControlLoopReplayer {
 public:
  typedef typename ControlLoop<T>::GoalType GoalType;
  typedef typename ControlLoop<T>::PositionType PositionType;
  typedef typename ControlLoop<T>::StatusType StatusType;
  typedef typename ControlLoop<T>::OutputType OutputType;

  // loop_group is where to send the messages out.
  // process_name is the name of the process which wrote the log messages in the
  // file(s).
  ControlLoopReplayer(T *loop_group, const ::std::string &process_name)
      : loop_group_(loop_group) {
    // Clear out any old messages which will confuse the code.
    loop_group_->status.FetchLatest();
    loop_group_->output.FetchLatest();

    replayer_.AddDirectQueueSender("wpilib_interface.DSReader",
                                   "joystick_state", ::aos::joystick_state);
    replayer_.AddDirectQueueSender("wpilib_interface.SensorReader",
                                   "robot_state", ::aos::robot_state);

    replayer_.AddHandler(
        process_name, "position",
        ::std::function<void(const PositionType &)>(::std::ref(position_)));
    replayer_.AddHandler(
        process_name, "output",
        ::std::function<void(const OutputType &)>(::std::ref(output_)));
    replayer_.AddHandler(
        process_name, "status",
        ::std::function<void(const StatusType &)>(::std::ref(status_)));
    // The timing of goal messages doesn't matter, and we don't need to look
    // back at them after running the loop.
    replayer_.AddDirectQueueSender(process_name, "goal", loop_group_->goal);
  }

  template <class QT>
  void AddDirectQueueSender(const ::std::string &process_name,
                            const ::std::string &log_message,
                            const ::aos::Queue<QT> &queue) {
    replayer_.AddDirectQueueSender<QT>(process_name, log_message, queue);
  }

  // Replays messages from a file.
  // filename can be straight from the command line; all sanity checking etc is
  // handled by this function.
  void ProcessFile(const char *filename);

 private:
  // A message handler which saves off messages and records whether it currently
  // has a new one or not.
  template <class S>
  class CaptureMessage {
   public:
    CaptureMessage() {}

    void operator()(const S &message) {
      CHECK(!have_new_message_);
      saved_message_ = message;
      have_new_message_ = true;
    }

    const S &saved_message() const { return saved_message_; }
    bool have_new_message() const { return have_new_message_; }
    void clear_new_message() { have_new_message_ = false; }

   private:
    S saved_message_;
    bool have_new_message_ = false;

    DISALLOW_COPY_AND_ASSIGN(CaptureMessage);
  };

  // Runs through the file currently loaded in replayer_.
  // Returns after going through the entire file.
  void DoProcessFile();

  T *const loop_group_;

  CaptureMessage<PositionType> position_;
  CaptureMessage<OutputType> output_;
  CaptureMessage<StatusType> status_;

  // The output that the loop sends for ZeroOutputs(). It might not actually be
  // all fields zeroed, so we pick the first one and remember it to compare.
  CaptureMessage<OutputType> zero_output_;

  ::aos::logging::linux_code::LogReplayer replayer_;
};

template <class T>
void ControlLoopReplayer<T>::ProcessFile(const char *filename) {
  int fd;
  if (strcmp(filename, "-") == 0) {
    fd = STDIN_FILENO;
  } else {
    fd = open(filename, O_RDONLY);
  }
  if (fd == -1) {
    PLOG(FATAL, "couldn't open file '%s' for reading", filename);
  }

  replayer_.OpenFile(fd);
  DoProcessFile();
  replayer_.CloseCurrentFile();

  PCHECK(close(fd));
}

template <class T>
void ControlLoopReplayer<T>::DoProcessFile() {
  while (true) {
    // Dig through messages until we get a status, which indicates the end of
    // the control loop cycle.
    while (!status_.have_new_message()) {
      if (replayer_.ProcessMessage()) return;
    }

    // Send out the position message (after adjusting the time offset) so the
    // loop will run right now.
    if (!position_.have_new_message()) {
      LOG(WARNING, "don't have a new position this cycle -> skipping\n");
      status_.clear_new_message();
      position_.clear_new_message();
      output_.clear_new_message();
      continue;
    }
    ::aos::time::OffsetToNow(position_.saved_message().sent_time);
    {
      auto position_message = loop_group_->position.MakeMessage();
      *position_message = position_.saved_message();
      CHECK(position_message.Send());
    }
    position_.clear_new_message();

    // Wait for the loop to finish running.
    loop_group_->status.FetchNextBlocking();

    // Point out if the status is different.
    if (!loop_group_->status->EqualsNoTime(status_.saved_message())) {
      LOG_STRUCT(WARNING, "expected status", status_.saved_message());
      LOG_STRUCT(WARNING, "got status", *loop_group_->status);
    }
    status_.clear_new_message();

    // Point out if the output is different. This is a lot more complicated than
    // for the status because there isn't always an output logged.
    bool loop_new_output = loop_group_->output.FetchLatest();
    if (output_.have_new_message()) {
      if (!loop_new_output) {
        LOG_STRUCT(WARNING, "no output, expected", output_.saved_message());
      } else if (!loop_group_->output->EqualsNoTime(output_.saved_message())) {
        LOG_STRUCT(WARNING, "expected output", output_.saved_message());
        LOG_STRUCT(WARNING, "got output", *loop_group_->output);
      }
    } else if (loop_new_output) {
      if (zero_output_.have_new_message()) {
        if (!loop_group_->output->EqualsNoTime(zero_output_.saved_message())) {
          LOG_STRUCT(WARNING, "expected null output",
                     zero_output_.saved_message());
          LOG_STRUCT(WARNING, "got output", *loop_group_->output);
        }
      } else {
        zero_output_(*loop_group_->output);
      }
    }
    output_.clear_new_message();
  }
}

}  // namespace controls
}  // namespace aos

#endif  // AOS_COMMON_CONTROLS_REPLAY_CONTROL_LOOP_H_
