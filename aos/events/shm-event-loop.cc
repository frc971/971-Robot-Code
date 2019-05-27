#include "aos/events/shm-event-loop.h"

#include <sys/timerfd.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <stdexcept>

#include "aos/logging/logging.h"
#include "aos/queue.h"

namespace aos {

ShmEventLoop::ShmEventLoop() : thread_state_(std::make_shared<ThreadState>()) {}

namespace {
class ShmFetcher : public RawFetcher {
 public:
  explicit ShmFetcher(RawQueue *queue) : queue_(queue) {}
  ~ShmFetcher() {
    if (msg_) {
      queue_->FreeMessage(msg_);
    }
  }

  bool FetchNext() override {
    const FetchValue *msg = static_cast<const FetchValue *>(
        queue_->ReadMessageIndex(RawQueue::kNonBlock, &index_));
    // Only update the internal pointer if we got a new message.
    if (msg != NULL) {
      queue_->FreeMessage(msg_);
      msg_ = msg;
      set_most_recent(msg_);
    }
    return msg != NULL;
  }

  bool Fetch() override {
    static constexpr Options<RawQueue> kOptions =
        RawQueue::kFromEnd | RawQueue::kNonBlock;
    const FetchValue *msg = static_cast<const FetchValue *>(
        queue_->ReadMessageIndex(kOptions, &index_));
    // Only update the internal pointer if we got a new message.
    if (msg != NULL && msg != msg_) {
      queue_->FreeMessage(msg_);
      msg_ = msg;
      set_most_recent(msg_);
      return true;
    }
    // The message has to get freed if we didn't use it (and
    // RawQueue::FreeMessage is ok to call on NULL).
    queue_->FreeMessage(msg);
    return false;
  }

 private:
  int index_ = 0;
  RawQueue *queue_;
  const FetchValue *msg_ = nullptr;
};

class ShmSender : public RawSender {
 public:
  explicit ShmSender(RawQueue *queue) : queue_(queue) {}

  aos::Message *GetMessage() override {
    return reinterpret_cast<aos::Message *>(queue_->GetMessage());
  }

  void Free(aos::Message *msg) override { queue_->FreeMessage(msg); }

  bool Send(aos::Message *msg) override {
    assert(queue_ != NULL);
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
class WatcherThreadState {
 public:
  WatcherThreadState(std::shared_ptr<ShmEventLoop::ThreadState> thread_state,
                     RawQueue *queue,
                     std::function<void(const aos::Message *message)> watcher)
      : thread_state_(std::move(thread_state)),
        queue_(queue),
        index_(0),
        watcher_(std::move(watcher)) {
    static constexpr Options<RawQueue> kOptions =
        RawQueue::kFromEnd | RawQueue::kNonBlock;
    const void *msg = queue_->ReadMessageIndex(kOptions, &index_);
    if (msg) {
      queue_->FreeMessage(msg);
    }
  }

  void Run() {
    thread_state_->WaitForStart();

    if (!thread_state_->is_running()) return;

    const void *msg = nullptr;
    while (true) {
      msg = queue_->ReadMessageIndex(RawQueue::kBlock, &index_);
      assert(msg != nullptr);

      {
        MutexLocker locker(&thread_state_->mutex_);
        if (!thread_state_->is_running()) break;

        watcher_(reinterpret_cast<const Message *>(msg));
        // watcher_ may have exited the event loop.
        if (!thread_state_->is_running()) break;
      }
      queue_->FreeMessage(msg);
    }

    queue_->FreeMessage(msg);
  }

 private:
  std::shared_ptr<ShmEventLoop::ThreadState> thread_state_;
  RawQueue *queue_;
  int32_t index_;

  std::function<void(const Message *message)> watcher_;
};

class TimerHandlerState : public TimerHandler {
 public:
  TimerHandlerState(std::shared_ptr<ShmEventLoop::ThreadState> thread_state,
                    ::std::function<void()> fn)
      : thread_state_(std::move(thread_state)), fn_(::std::move(fn)) {
    fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    PCHECK(fd_ != -1);
  }

  ~TimerHandlerState() {
    PCHECK(close(fd_) == 0);
  }

  void Setup(monotonic_clock::time_point base,
             monotonic_clock::duration repeat_offset) override {
    struct itimerspec new_value;
    new_value.it_interval = ::aos::time::to_timespec(repeat_offset);
    new_value.it_value = ::aos::time::to_timespec(base);
    PCHECK(timerfd_settime(fd_, TFD_TIMER_ABSTIME, &new_value, nullptr) == 0);
  }

  void Disable() override {
    // Disarm the timer by feeding zero values
    Setup(::aos::monotonic_clock::epoch(), ::aos::monotonic_clock::zero());
  }

  void Run() {
    thread_state_->WaitForStart();

    while (true) {
      uint64_t buf;
      ssize_t result = read(fd_, &buf, sizeof(buf));
      PCHECK(result != -1);
      CHECK_EQ(result, static_cast<int>(sizeof(buf)));

      {
        MutexLocker locker(&thread_state_->mutex_);
        if (!thread_state_->is_running()) break;
        fn_();
        // fn_ may have exited the event loop.
        if (!thread_state_->is_running()) break;
      }
    }
  }

 private:
  std::shared_ptr<ShmEventLoop::ThreadState> thread_state_;

  // File descriptor for the timer
  int fd_;

  // Function to be run on the thread
  ::std::function<void()> fn_;
};
}  // namespace internal

std::unique_ptr<RawFetcher> ShmEventLoop::MakeRawFetcher(
    const std::string &path, const QueueTypeInfo &type) {
  return std::unique_ptr<RawFetcher>(new ShmFetcher(
      RawQueue::Fetch(path.c_str(), type.size, type.hash, type.queue_length)));
}

std::unique_ptr<RawSender> ShmEventLoop::MakeRawSender(
    const std::string &path, const QueueTypeInfo &type) {
  Take(path);
  return std::unique_ptr<RawSender>(new ShmSender(
      RawQueue::Fetch(path.c_str(), type.size, type.hash, type.queue_length)));
}

void ShmEventLoop::MakeRawWatcher(
    const std::string &path, const QueueTypeInfo &type,
    std::function<void(const Message *message)> watcher) {
  Take(path);
  auto *state = new internal::WatcherThreadState(
      thread_state_,
      RawQueue::Fetch(path.c_str(), type.size, type.hash, type.queue_length),
      std::move(watcher));

  std::thread thread([state] {
    state->Run();
    delete state;
  });
  thread.detach();
}

TimerHandler *ShmEventLoop::AddTimer(::std::function<void()> callback) {
  internal::TimerHandlerState *timer =
      new internal::TimerHandlerState(thread_state_, ::std::move(callback));

  ::std::thread t([timer] {
    timer->Run();
    delete timer;
  });
  t.detach();

  return timer;
}

void ShmEventLoop::OnRun(std::function<void()> on_run) {
  on_run_.push_back(std::move(on_run));
}

void ShmEventLoop::Run() {
  set_is_running(true);
  for (const auto &run : on_run_) run();
  // TODO(austin): epoll event loop in main thread (if needed), and async safe
  // quit handler.
  thread_state_->Run();
}

void ShmEventLoop::ThreadState::Run() {
  MutexLocker locker(&mutex_);
  loop_running_ = true;
  if (loop_finished_) ::aos::Die("Cannot restart an ShmEventLoop()");
  loop_running_cond_.Broadcast();
  while (loop_running_) {
    if (loop_running_cond_.Wait()) {
      ::aos::Die("ShmEventLoop mutex lock problem.\n");
    }
  }
}

void ShmEventLoop::ThreadState::WaitForStart() {
  MutexLocker locker(&mutex_);
  while (!(loop_running_ || loop_finished_)) {
    if (loop_running_cond_.Wait()) {
      ::aos::Die("ShmEventLoop mutex lock problem.\n");
    }
  }
}

void ShmEventLoop::Exit() {
  set_is_running(false);
  thread_state_->Exit();
}

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

void ShmEventLoop::Take(const std::string &path) {
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
