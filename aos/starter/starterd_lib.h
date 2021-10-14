#ifndef AOS_STARTER_STARTERD_LIB_H_
#define AOS_STARTER_STARTERD_LIB_H_

#include <sys/signalfd.h>
#include <sys/wait.h>

#include <csignal>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/ipc_lib/signalfd.h"
#include "aos/macros.h"
#include "aos/starter/starter_generated.h"
#include "aos/starter/starter_rpc_generated.h"

namespace aos {
namespace starter {

// RAII Pipe for sending individual ints between reader and writer.
class ScopedPipe {
 public:
  class ScopedReadPipe;
  class ScopedWritePipe;

  static std::tuple<ScopedReadPipe, ScopedWritePipe> MakePipe();

  virtual ~ScopedPipe();

  int fd() const { return fd_; }

 private:
  ScopedPipe(int fd = -1);

  int fd_;

  ScopedPipe(const ScopedPipe &) = delete;
  ScopedPipe &operator=(const ScopedPipe &) = delete;
  ScopedPipe(ScopedPipe &&);
  ScopedPipe &operator=(ScopedPipe &&);
};

class ScopedPipe::ScopedReadPipe : public ScopedPipe {
 public:
  std::optional<uint32_t> Read();

 private:
  using ScopedPipe::ScopedPipe;

  friend class ScopedPipe;
};

class ScopedPipe::ScopedWritePipe : public ScopedPipe {
 public:
  void Write(uint32_t data);

 private:
  using ScopedPipe::ScopedPipe;

  friend class ScopedPipe;
};

// Manages a running process, allowing starting and stopping, and restarting
// automatically.
class Application {
 public:
  Application(const aos::Application *application,
              aos::ShmEventLoop *event_loop);

  flatbuffers::Offset<aos::starter::ApplicationStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *builder);

  // Returns the last pid of this process. -1 if not started yet.
  pid_t get_pid() const { return pid_; }

  // Handles a SIGCHLD signal received by the parent. Does nothing if this
  // process was not the target. Returns true if this Application should be
  // removed.
  bool MaybeHandleSignal();

  // Handles a command. May do nothing if application is already in the desired
  // state.
  void HandleCommand(aos::starter::Command cmd);

  void Start() { HandleCommand(aos::starter::Command::START); }

  void Stop() { HandleCommand(aos::starter::Command::STOP); }

  void Terminate();

  void set_args(
      const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>
          &args);

  bool autostart() const { return autostart_; }

 private:
  void DoStart();

  void DoStop(bool restart);

  void QueueStart();

  // Copy flatbuffer vector of strings to vector of std::string.
  static std::vector<std::string> FbsVectorToVector(
      const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> &v);

  static std::optional<uid_t> FindUid(const char *name);
  static std::optional<gid_t> FindPrimaryGidForUser(const char *name);

  // Next unique id for all applications
  static inline uint64_t next_id_ = 0;

  std::string name_;
  std::string path_;
  std::vector<char *> args_;
  std::optional<uid_t> user_;
  std::optional<gid_t> group_;

  pid_t pid_ = -1;
  ScopedPipe::ScopedReadPipe read_pipe_;
  ScopedPipe::ScopedWritePipe write_pipe_;
  uint64_t id_;
  int exit_code_ = 0;
  aos::monotonic_clock::time_point start_time_, exit_time_;
  bool queue_restart_ = false;
  bool terminating_ = false;
  bool autostart_ = true;

  aos::starter::State status_ = aos::starter::State::STOPPED;
  aos::starter::LastStopReason stop_reason_ =
      aos::starter::LastStopReason::STOP_REQUESTED;

  aos::ShmEventLoop *event_loop_;
  aos::TimerHandler *start_timer_, *restart_timer_, *stop_timer_;

  DISALLOW_COPY_AND_ASSIGN(Application);
};

// Registers a signalfd listener with the given event loop and calls callback
// whenever a signal is received.
class SignalListener {
 public:
  SignalListener(aos::ShmEventLoop *loop,
                 std::function<void(signalfd_siginfo)> callback);

  ~SignalListener();

 private:
  aos::ShmEventLoop *loop_;
  std::function<void(signalfd_siginfo)> callback_;
  aos::ipc_lib::SignalFd signalfd_;

  DISALLOW_COPY_AND_ASSIGN(SignalListener);
};

class Starter {
 public:
  Starter(const aos::Configuration *event_loop_config);

  // Inserts a new application from config. Returns the inserted application if
  // it was successful, otherwise nullptr if an application already exists
  // with the given name.
  Application *AddApplication(const aos::Application *application);

  // Runs the event loop and starts all applications
  void Run();

  void Cleanup();

 private:
  // Signals which indicate starter has died
  static const inline std::vector<int> kStarterDeath = {
      SIGHUP,  SIGINT,  SIGQUIT, SIGILL, SIGABRT, SIGFPE,
      SIGSEGV, SIGPIPE, SIGTERM, SIGBUS, SIGXCPU};

  void OnSignal(signalfd_siginfo signal);

  void SendStatus();

  const std::string config_path_;
  const aos::Configuration *config_msg_;

  aos::ShmEventLoop event_loop_;
  aos::Sender<aos::starter::Status> status_sender_;
  aos::TimerHandler *status_timer_;
  aos::TimerHandler *cleanup_timer_;

  std::unordered_map<std::string, Application> applications_;

  // Set to true on cleanup to block rpc commands and ensure cleanup only
  // happens once.
  bool exiting_ = false;

  SignalListener listener_;

  DISALLOW_COPY_AND_ASSIGN(Starter);
};

}  // namespace starter
}  // namespace aos

#endif  // AOS_STARTER_STARTERD_LIB_H_
