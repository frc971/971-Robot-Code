#ifndef AOS_STARTER_STARTERD_LIB_H_
#define AOS_STARTER_STARTERD_LIB_H_

#include <sys/signalfd.h>

#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/ftrace.h"
#include "aos/ipc_lib/memory_mapped_queue.h"
#include "aos/macros.h"
#include "aos/starter/starter_generated.h"
#include "aos/starter/starter_rpc_generated.h"
#include "aos/starter/subprocess.h"
#include "aos/util/top.h"

namespace aos::starter {

const aos::Channel *StatusChannelForNode(const aos::Configuration *config,
                                         const aos::Node *node);
const aos::Channel *StarterRpcChannelForNode(const aos::Configuration *config,
                                             const aos::Node *node);

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

  // EventLoop that we use for running the code. Mostly exposed for testing
  // purposes.
  EventLoop *event_loop() { return &event_loop_; }

 private:
  // Signals which indicate starter has died
  static const inline std::vector<int> kStarterDeath = {
      SIGHUP,  SIGINT,  SIGQUIT, SIGILL, SIGABRT, SIGFPE,
      SIGSEGV, SIGPIPE, SIGTERM, SIGBUS, SIGXCPU};

  void OnSignal(signalfd_siginfo signal);
  void HandleStarterRpc(const StarterRpc &command);

  // Handles any potential state change in the child applications.
  // In particular, sends the Status message if it wouldn't exceed the rate
  // limit.
  void HandleStateChange();

  // Called periodically to run through the timing report fetcher and alert all
  // the Application's to the new messages.
  void ServiceTimingReportFetcher(int elapsed_cycles);

  void SendStatus();

  // Creates a MemoryMappedQueue for the given channel, to pre-allocate shared
  // memory to give this process credit for the memory instead of any other
  // process that accesses it.
  void AddChannel(const aos::Channel *channel);

  const std::string config_path_;
  const aos::Configuration *config_msg_;

  aos::ShmEventLoop event_loop_;
  aos::Sender<aos::starter::Status> status_sender_;
  aos::PhasedLoopHandler *status_timer_;
  aos::TimerHandler *cleanup_timer_;

  aos::Ftrace ftrace_;

  int status_count_ = 0;
  const int max_status_count_;

  aos::Fetcher<aos::timing::Report> timing_report_fetcher_;

  std::unordered_map<std::string, Application> applications_;

  // Lock and list of all the queues.  This makes it so we can initialize the
  // queues in parallel, and also so starterd owns the memory for all the
  // queues from cgroup's point of view.
  std::mutex queue_mutex_;
  std::vector<std::unique_ptr<aos::ipc_lib::MemoryMappedQueue>> shm_queues_;

  // Capture the --shm_base flag at construction time.  This makes it much
  // easier to make different shared memory regions for doing things like
  // multi-node tests.
  std::string shm_base_;

  // Set to true on cleanup to block rpc commands and ensure cleanup only
  // happens once.
  bool exiting_ = false;

  SignalListener listener_;

  util::Top top_;

  DISALLOW_COPY_AND_ASSIGN(Starter);
};

}  // namespace aos::starter

#endif  // AOS_STARTER_STARTERD_LIB_H_
