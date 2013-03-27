#include "signal_handler.h"

#include <boost/thread/locks.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <google/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <memory>
#include <queue>
#include <unistd.h>

#include "thread.h"

// Boolean signaling if the handler has been set up or not.
static bool pipe_has_been_initialized = false;

// Pipe that the signal handler writes the signal number to
static int handler_write_pipe;
// Pipe that the handler thread reads the signal number from.
static int thread_read_pipe;
// Mutex to lock the signal handler dictionary.
static boost::mutex signal_dict_lock;
// Signal handler dictionary.
static std::unique_ptr<std::map<int, boost::function<void(int)> > > signal_dict;

// Thread which reads signal numbers from the pipe and handles them.
class SignalHandlerThread: public Thread {
 public:
  void operator()() {
    while (true) {
      int signal_number;
      LOG(INFO) << "Back to waiting.";
      int size = read(thread_read_pipe, &signal_number, sizeof(int));
      if (size == -1) {
        continue;
      }
      CHECK_EQ(size, sizeof(int))
          << ": Read the wrong number of bytes when receiving a signal.";
      LOG(INFO) << "Got signal " << signal_number;

      boost::function<void(int)> function;
      if (GetFromMap(*signal_dict, signal_number, &function)) {
        function(signal_number);
      }
    }
  }
};

static std::unique_ptr<SignalHandlerThread> signal_handler_thread;
static std::unique_ptr<boost::thread> signal_handler_thread_;

// Simple signal handler which writes the signal number to the pipe.
static void signal_handler(int signal_number) {
  CHECK_EQ(write(handler_write_pipe, &signal_number, sizeof(int)),
           sizeof(int));
}

void RegisterSignalHandler(int signal_number,
                           boost::function<void(int)> function){
  if (!pipe_has_been_initialized) {
    int pipefd[2];
    CHECK(!pipe2(pipefd, 0))
        << ": Failed to create pipes for signal handler.";
    thread_read_pipe = pipefd[0];
    handler_write_pipe = pipefd[1];
    signal_handler_thread.reset(new SignalHandlerThread());
    signal_handler_thread_.reset(
        new boost::thread(boost::ref(*signal_handler_thread)));
    signal_dict.reset(new std::map<int, boost::function<void(int)> >());
  }
  struct sigaction new_action, old_action;

  new_action.sa_handler = &signal_handler;
  sigemptyset(&new_action.sa_mask);

  boost::lock_guard<boost::mutex> lockguard(signal_dict_lock);

  InsertIntoMap(signal_dict.get(), signal_number, function);

  sigaction(signal_number, &new_action, NULL);
}
