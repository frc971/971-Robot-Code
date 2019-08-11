#include "aos/events/shm-event-loop.h"

#include <sys/timerfd.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <stdexcept>

#include "aos/events/epoll.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/queue.h"
#include "aos/util/phased_loop.h"

namespace aos {

ShmEventLoop::ShmEventLoop() {}

namespace {

namespace chrono = ::std::chrono;

class ShmFetcher : public RawFetcher {
 public:
  explicit ShmFetcher(RawQueue *queue) : queue_(queue) {
    // Move index_ to point to the end of the queue as it is at construction
    // time.  Also grab the oldest message but don't expose it to the user yet.
    static constexpr Options<RawQueue> kOptions =
        RawQueue::kFromEnd | RawQueue::kNonBlock;
    msg_ = static_cast<const FetchValue *>(
        queue_->ReadMessageIndex(kOptions, &index_));
  }
  ~ShmFetcher() {
    if (msg_) {
      queue_->FreeMessage(msg_);
    }
  }

  bool FetchNext() override {
    const FetchValue *msg = static_cast<const FetchValue *>(
        queue_->ReadMessageIndex(RawQueue::kNonBlock, &index_));
    // Only update the internal pointer if we got a new message.
    if (msg != nullptr) {
      queue_->FreeMessage(msg_);
      msg_ = msg;
      set_most_recent(msg_);
    }
    return msg != nullptr;
  }

  bool Fetch() override {
    static constexpr Options<RawQueue> kOptions =
        RawQueue::kFromEnd | RawQueue::kNonBlock;
    const FetchValue *msg = static_cast<const FetchValue *>(
        queue_->ReadMessageIndex(kOptions, &index_));
    // Only update the internal pointer if we got a new message.
    if (msg != nullptr && msg != msg_) {
      queue_->FreeMessage(msg_);
      msg_ = msg;
      set_most_recent(msg_);
      return true;
    } else {
      // The message has to get freed if we didn't use it (and
      // RawQueue::FreeMessage is ok to call on nullptr).
      queue_->FreeMessage(msg);

      // We have a message from construction time.  Give it to the user now.
      if (msg_ != nullptr && most_recent() != msg_) {
        set_most_recent(msg_);
        return true;
      } else {
        return false;
      }
    }
  }

 private:
  int index_ = 0;
  RawQueue *queue_;
  const FetchValue *msg_ = nullptr;
};

class ShmSender : public RawSender {
 public:
  explicit ShmSender(RawQueue *queue) : queue_(queue) {}

  ::aos::Message *GetMessage() override {
    return reinterpret_cast<::aos::Message *>(queue_->GetMessage());
  }

  void Free(::aos::Message *msg) override { queue_->FreeMessage(msg); }

  bool Send(::aos::Message *msg) override {
    assert(queue_ != nullptr);
    {
      // TODO(austin): This lets multiple senders reorder messages since time
      // isn't acquired with a lock held.
      if (msg->sent_time == monotonic_clock::min_time) {
        msg->sent_time = monotonic_clock::now();
      }
    }
    return queue_->WriteMessage(msg, RawQueue::kOverride);
  }

  const char *name() const override { return queue_->name(); }

 private:
  RawQueue *queue_;
};

}  // namespace

namespace internal {

// Class to manage the state for a Watcher.
class WatcherThreadState {
 public:
  WatcherThreadState(
      ShmEventLoop::ThreadState *thread_state, RawQueue *queue,
      ::std::function<void(const ::aos::Message *message)> watcher)
      : thread_state_(thread_state),
        queue_(queue),
        index_(0),
        watcher_(::std::move(watcher)) {}

  ~WatcherThreadState() {
    // Only kill the thread if it is running.
    if (running_) {
      // TODO(austin): CHECK that we aren't RT here.

      // Try joining.  If we fail, we weren't asleep on the condition in the
      // queue.  So hit it again and again until that's true.
      struct timespec end_time;
      PCHECK(clock_gettime(CLOCK_REALTIME, &end_time) == 0);
      while (true) {
        void *retval = nullptr;
        end_time.tv_nsec += 100000000;
        if (end_time.tv_nsec > 1000000000L) {
          end_time.tv_nsec -= 1000000000L;
          ++end_time.tv_sec;
        }
        int ret = pthread_timedjoin_np(pthread_, &retval, &end_time);
        if (ret == ETIMEDOUT) continue;
        PCHECK(ret == 0);
        break;
      }
    }
  }

  // Starts the thread and waits until it is running.
  void Start() {
    PCHECK(pthread_create(&pthread_, nullptr, &StaticRun, this) == 0);
    IPCRecursiveMutexLocker locker(&thread_started_mutex_);
    if (locker.owner_died()) ::aos::Die("Owner died");
    while (!running_) {
      CHECK(!thread_started_condition_.Wait());
    }
  }

  void GrabQueueIndex() {
    // Right after we are signaled to start, point index to the current index
    // so we don't read any messages received before now.  Otherwise we will
    // get a significantly delayed read.
    static constexpr Options<RawQueue> kOptions =
        RawQueue::kFromEnd | RawQueue::kNonBlock;
    const void *msg = queue_->ReadMessageIndex(kOptions, &index_);
    if (msg) {
      queue_->FreeMessage(msg);
    }
  }

 private:
  // Runs Run given a WatcherThreadState as the argument.  This is an adapter
  // between pthreads and Run.
  static void *StaticRun(void *arg) {
    WatcherThreadState *watcher_thread_state =
        reinterpret_cast<WatcherThreadState *>(arg);
    watcher_thread_state->Run();
    return nullptr;
  }

  // Runs the watcher callback on new messages.
  void Run() {
    ::aos::SetCurrentThreadName(thread_state_->name() + ".watcher");

    // Signal the main thread that we are now ready.
    thread_state_->MaybeSetCurrentThreadRealtimePriority();
    {
      IPCRecursiveMutexLocker locker(&thread_started_mutex_);
      if (locker.owner_died()) ::aos::Die("Owner died");
      running_ = true;
      thread_started_condition_.Broadcast();
    }

    // Wait for the global start before handling events.
    thread_state_->WaitForStart();

    // Bail immediately if we are supposed to stop.
    if (!thread_state_->is_running()) {
      ::aos::UnsetCurrentThreadRealtimePriority();
      return;
    }

    const void *msg = nullptr;
    while (true) {
      msg = queue_->ReadMessageIndex(RawQueue::kBlock, &index_,
                                     chrono::seconds(1));
      // We hit a timeout.  Confirm that we should be running and retry.  Note,
      // is_running is threadsafe (it's an atomic underneath).  Worst case, we
      // check again in a second.
      if (msg == nullptr) {
        if (!thread_state_->is_running()) break;
        continue;
      }

      {
        // Grab the lock so that only one callback can be called at a time.
        MutexLocker locker(&thread_state_->mutex_);
        if (!thread_state_->is_running()) break;

        watcher_(reinterpret_cast<const Message *>(msg));
        // watcher_ may have exited the event loop.
        if (!thread_state_->is_running()) break;
      }
      // Drop the reference.
      queue_->FreeMessage(msg);
    }

    // And drop the last reference.
    queue_->FreeMessage(msg);
    // Now that everything is cleaned up, drop RT priority before destroying the
    // thread.
    ::aos::UnsetCurrentThreadRealtimePriority();
  }
  pthread_t pthread_;
  ShmEventLoop::ThreadState *thread_state_;
  RawQueue *queue_;
  int32_t index_;
  bool running_ = false;

  ::std::function<void(const Message *message)> watcher_;

  // Mutex and condition variable used to wait until the thread is started
  // before going RT.
  ::aos::Mutex thread_started_mutex_;
  ::aos::Condition thread_started_condition_{&thread_started_mutex_};
};

// Adapter class to adapt a timerfd to a TimerHandler.
// The part of the API which is accessed by the TimerHandler interface needs to
// be threadsafe.  This means Setup and Disable.
class TimerHandlerState : public TimerHandler {
 public:
  TimerHandlerState(ShmEventLoop *shm_event_loop, ::std::function<void()> fn)
      : shm_event_loop_(shm_event_loop), fn_(::std::move(fn)) {
    shm_event_loop_->epoll_.OnReadable(timerfd_.fd(), [this]() {
      MutexLocker locker(&shm_event_loop_->thread_state_.mutex_);
      timerfd_.Read();
      fn_();
    });
  }

  ~TimerHandlerState() { shm_event_loop_->epoll_.DeleteFd(timerfd_.fd()); }

  void Setup(monotonic_clock::time_point base,
             monotonic_clock::duration repeat_offset) override {
    // SetTime is threadsafe already.
    timerfd_.SetTime(base, repeat_offset);
  }

  void Disable() override {
    // Disable is also threadsafe already.
    timerfd_.Disable();
  }

 private:
  ShmEventLoop *shm_event_loop_;

  TimerFd timerfd_;

  // Function to be run on the thread
  ::std::function<void()> fn_;
};

// Adapter class to the timerfd and PhasedLoop.
// The part of the API which is accessed by the PhasedLoopHandler interface
// needs to be threadsafe.  This means set_interval_and_offset
class PhasedLoopHandler : public ::aos::PhasedLoopHandler {
 public:
  PhasedLoopHandler(ShmEventLoop *shm_event_loop, ::std::function<void(int)> fn,
                    const monotonic_clock::duration interval,
                    const monotonic_clock::duration offset)
      : shm_event_loop_(shm_event_loop),
        phased_loop_(interval, shm_event_loop_->monotonic_now(), offset),
        fn_(::std::move(fn)) {
    shm_event_loop_->epoll_.OnReadable(timerfd_.fd(), [this]() {
      MutexLocker locker(&shm_event_loop_->thread_state_.mutex_);
      {
        MutexLocker locker(&mutex_);
        timerfd_.Read();
      }
      // Call the function.  To avoid needing a recursive mutex, drop the lock
      // before running the function.
      fn_(cycles_elapsed_);
      {
        MutexLocker locker(&mutex_);
        Reschedule();
      }
    });
  }

  ~PhasedLoopHandler() { shm_event_loop_->epoll_.DeleteFd(timerfd_.fd()); }

  void set_interval_and_offset(
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset) override {
    MutexLocker locker(&mutex_);
    phased_loop_.set_interval_and_offset(interval, offset);
  }

  void Startup() {
    MutexLocker locker(&mutex_);
    phased_loop_.Reset(shm_event_loop_->monotonic_now());
    Reschedule();
  }

 private:
  // Reschedules the timer.  Must be called with the mutex held.
  void Reschedule() {
    cycles_elapsed_ = phased_loop_.Iterate(shm_event_loop_->monotonic_now());
    timerfd_.SetTime(phased_loop_.sleep_time(), ::aos::monotonic_clock::zero());
  }

  ShmEventLoop *shm_event_loop_;

  // Mutex to protect access to the timerfd_ (not strictly necessary), and the
  // phased_loop (necessary).
  ::aos::Mutex mutex_;

  TimerFd timerfd_;
  time::PhasedLoop phased_loop_;

  int cycles_elapsed_ = 1;

  // Function to be run
  const ::std::function<void(int)> fn_;
};
}  // namespace internal

::std::unique_ptr<RawFetcher> ShmEventLoop::MakeRawFetcher(
    const ::std::string &path, const QueueTypeInfo &type) {
  return ::std::unique_ptr<RawFetcher>(new ShmFetcher(
      RawQueue::Fetch(path.c_str(), type.size, type.hash, type.queue_length)));
}

::std::unique_ptr<RawSender> ShmEventLoop::MakeRawSender(
    const ::std::string &path, const QueueTypeInfo &type) {
  Take(path);
  return ::std::unique_ptr<RawSender>(new ShmSender(
      RawQueue::Fetch(path.c_str(), type.size, type.hash, type.queue_length)));
}

void ShmEventLoop::MakeRawWatcher(
    const ::std::string &path, const QueueTypeInfo &type,
    ::std::function<void(const Message *message)> watcher) {
  Take(path);
  ::std::unique_ptr<internal::WatcherThreadState> state(
      new internal::WatcherThreadState(
          &thread_state_, RawQueue::Fetch(path.c_str(), type.size, type.hash,
                                          type.queue_length),
          std::move(watcher)));
  watchers_.push_back(::std::move(state));
}

TimerHandler *ShmEventLoop::AddTimer(::std::function<void()> callback) {
  ::std::unique_ptr<internal::TimerHandlerState> timer(
      new internal::TimerHandlerState(this, ::std::move(callback)));

  timers_.push_back(::std::move(timer));

  return timers_.back().get();
}

PhasedLoopHandler *ShmEventLoop::AddPhasedLoop(
    ::std::function<void(int)> callback,
    const monotonic_clock::duration interval,
    const monotonic_clock::duration offset) {
  ::std::unique_ptr<internal::PhasedLoopHandler> phased_loop(
      new internal::PhasedLoopHandler(this, ::std::move(callback), interval,
                                      offset));

  phased_loops_.push_back(::std::move(phased_loop));

  return phased_loops_.back().get();
}

void ShmEventLoop::OnRun(::std::function<void()> on_run) {
  on_run_.push_back(::std::move(on_run));
}

void ShmEventLoop::set_name(const char *name) { thread_state_.name_ = name; }

void ShmEventLoop::Run() {
  // Start all the watcher threads.
  for (::std::unique_ptr<internal::WatcherThreadState> &watcher : watchers_) {
    watcher->Start();
  }

  ::aos::SetCurrentThreadName(thread_state_.name());

  // Now, all the threads are up.  Lock everything into memory and go RT.
  if (thread_state_.priority_ != -1) {
    ::aos::InitRT();
  }
  thread_state_.MaybeSetCurrentThreadRealtimePriority();
  set_is_running(true);

  // Now that we are realtime (but before the OnRun handlers run), snap the
  // queue index.
  for (::std::unique_ptr<internal::WatcherThreadState> &watcher : watchers_) {
    watcher->GrabQueueIndex();
  }

  // Now that we are RT, run all the OnRun handlers.
  for (const auto &run : on_run_) {
    run();
  }

  // Start up all the phased loops.
  for (::std::unique_ptr<internal::PhasedLoopHandler> &phased_loop :
       phased_loops_) {
    phased_loop->Startup();
  }
  // TODO(austin): We don't need a separate watcher thread if there are only
  // watchers and fetchers.  Could lazily create the epoll loop and pick a
  // victim watcher to run in this thread.
  // Trigger all the threads to start now.
  thread_state_.Start();

  // And start our main event loop which runs all the timers and handles Quit.
  epoll_.Run();

  // Once epoll exits, there is no useful nonrt work left to do.
  set_is_running(false);

  // Signal all the watcher threads to exit.  After this point, no more
  // callbacks will be handled.
  thread_state_.Exit();

  // Nothing time or synchronization critical needs to happen after this point.
  // Drop RT priority.
  ::aos::UnsetCurrentThreadRealtimePriority();

  // The watcher threads get cleaned up in the destructor.
}

void ShmEventLoop::ThreadState::Start() {
  MutexLocker locker(&mutex_);
  loop_running_ = true;
  if (loop_finished_) ::aos::Die("Cannot restart an ShmEventLoop()");
  loop_running_cond_.Broadcast();
}

void ShmEventLoop::ThreadState::WaitForStart() {
  MutexLocker locker(&mutex_);
  while (!(loop_running_ || loop_finished_)) {
    Condition::WaitResult wait_result =
        loop_running_cond_.WaitTimed(chrono::milliseconds(1000));
    if (wait_result == Condition::WaitResult::kOwnerDied) {
      ::aos::Die("ShmEventLoop mutex lock problem.\n");
    }
  }
}

void ShmEventLoop::ThreadState::MaybeSetCurrentThreadRealtimePriority() {
  if (priority_ != -1) {
    ::aos::SetCurrentThreadRealtimePriority(priority_);
  }
}

void ShmEventLoop::Exit() { epoll_.Quit(); }

void ShmEventLoop::ThreadState::Exit() {
  IPCRecursiveMutexLocker locker(&mutex_);
  if (locker.owner_died()) ::aos::Die("Owner died");
  loop_running_ = false;
  loop_finished_ = true;
  loop_running_cond_.Broadcast();
}

ShmEventLoop::~ShmEventLoop() {
  if (is_running()) {
    ::aos::Die("ShmEventLoop destroyed while running\n");
  }
}

void ShmEventLoop::Take(const ::std::string &path) {
  if (is_running()) {
    ::aos::Die("Cannot add new objects while running.\n");
  }

  const auto prior = ::std::find(taken_.begin(), taken_.end(), path);
  if (prior != taken_.end()) {
    ::aos::Die("%s is already being used.", path.c_str());
  } else {
    taken_.emplace_back(path);
  }
}

}  // namespace aos
