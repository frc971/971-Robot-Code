#include "glog/logging.h"

#include "aos/events/shm_event_loop.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <stdexcept>

#include "aos/events/epoll.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/realtime.h"
#include "aos/util/phased_loop.h"

DEFINE_string(shm_base, "/dev/shm/aos",
              "Directory to place queue backing mmaped files in.");
DEFINE_uint32(permissions, 0770,
              "Permissions to make shared memory files and folders.");

namespace aos {

std::string ShmFolder(const Channel *channel) {
  CHECK(channel->has_name());
  CHECK_EQ(channel->name()->string_view()[0], '/');
  return FLAGS_shm_base + channel->name()->str() + "/";
}
std::string ShmPath(const Channel *channel) {
  CHECK(channel->has_type());
  return ShmFolder(channel) + channel->type()->str() + ".v0";
}

class MMapedQueue {
 public:
  MMapedQueue(const Channel *channel) {
    std::string path = ShmPath(channel);

    // TODO(austin): Pull these out into the config if there is a need.
    config_.num_watchers = 10;
    config_.num_senders = 10;
    config_.queue_size = 2 * channel->frequency();
    config_.message_data_size = channel->max_size();

    size_ = ipc_lib::LocklessQueueMemorySize(config_);

    MkdirP(path);

    // There are 2 cases.  Either the file already exists, or it does not
    // already exist and we need to create it.  Start by trying to create it. If
    // that fails, the file has already been created and we can open it
    // normally..  Once the file has been created it wil never be deleted.
    fd_ = open(path.c_str(), O_RDWR | O_CREAT | O_EXCL,
               O_CLOEXEC | FLAGS_permissions);
    if (fd_ == -1 && errno == EEXIST) {
      VLOG(1) << path << " already created.";
      // File already exists.
      fd_ = open(path.c_str(), O_RDWR, O_CLOEXEC);
      PCHECK(fd_ != -1) << ": Failed to open " << path;
      while (true) {
        struct stat st;
        PCHECK(fstat(fd_, &st) == 0);
        if (st.st_size != 0) {
          CHECK_EQ(static_cast<size_t>(st.st_size), size_)
              << ": Size of " << path
              << " doesn't match expected size of backing queue file.  Did the "
                 "queue definition change?";
          break;
        } else {
          // The creating process didn't get around to it yet.  Give it a bit.
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          VLOG(1) << path << " is zero size, waiting";
        }
      }
    } else {
      VLOG(1) << "Created " << path;
      PCHECK(fd_ != -1) << ": Failed to open " << path;
      PCHECK(ftruncate(fd_, size_) == 0);
    }

    data_ = mmap(NULL, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    PCHECK(data_ != MAP_FAILED);

    ipc_lib::InitializeLocklessQueueMemory(memory(), config_);
  }

  ~MMapedQueue() {
    PCHECK(munmap(data_, size_) == 0);
    PCHECK(close(fd_) == 0);
  }

  ipc_lib::LocklessQueueMemory *memory() const {
    return reinterpret_cast<ipc_lib::LocklessQueueMemory *>(data_);
  }

  const ipc_lib::LocklessQueueConfiguration &config() const {
    return config_;
  }

 private:
  void MkdirP(std::string_view path) {
    struct stat st;
    auto last_slash_pos = path.find_last_of("/");

    std::string folder(last_slash_pos == std::string_view::npos
                           ? std::string_view("")
                           : path.substr(0, last_slash_pos));
    if (stat(folder.c_str(), &st) == -1) {
      PCHECK(errno == ENOENT);
      CHECK_NE(folder, "") << ": Base path doesn't exist";
      MkdirP(folder);
      VLOG(1) << "Creating " << folder;
      PCHECK(mkdir(folder.c_str(), FLAGS_permissions) == 0);
    }
  }

  ipc_lib::LocklessQueueConfiguration config_;

  int fd_;

  size_t size_;
  void *data_;
};

// Returns the portion of the path after the last /.
std::string_view Filename(std::string_view path) {
  auto last_slash_pos = path.find_last_of("/");

  return last_slash_pos == std::string_view::npos
             ? path
             : path.substr(last_slash_pos + 1, path.size());
}

ShmEventLoop::ShmEventLoop(const Configuration *configuration)
    : EventLoop(configuration), name_(Filename(program_invocation_name)) {}

namespace {

namespace chrono = ::std::chrono;

class ShmFetcher : public RawFetcher {
 public:
  explicit ShmFetcher(const Channel *channel)
      : RawFetcher(channel),
        lockless_queue_memory_(channel),
        lockless_queue_(lockless_queue_memory_.memory(),
                        lockless_queue_memory_.config()),
        data_storage_(static_cast<AlignedChar *>(aligned_alloc(
                          alignof(AlignedChar), channel->max_size())),
                      &free) {
    context_.data = nullptr;
    // Point the queue index at the next index to read starting now.  This
    // makes it such that FetchNext will read the next message sent after
    // the fetcher is created.
    PointAtNextQueueIndex();
  }

  ~ShmFetcher() { data_ = nullptr; }

  // Points the next message to fetch at the queue index which will be
  // populated next.
  void PointAtNextQueueIndex() {
    actual_queue_index_ = lockless_queue_.LatestQueueIndex();
    if (!actual_queue_index_.valid()) {
      // Nothing in the queue.  The next element will show up at the 0th
      // index in the queue.
      actual_queue_index_ =
          ipc_lib::QueueIndex::Zero(lockless_queue_.queue_size());
    } else {
      actual_queue_index_ = actual_queue_index_.Increment();
    }
  }

  bool FetchNext() override {
    // TODO(austin): Write a test which starts with nothing in the queue,
    // and then calls FetchNext() after something is sent.
    // TODO(austin): Get behind and make sure it dies both here and with
    // Fetch.
    ipc_lib::LocklessQueue::ReadResult read_result = lockless_queue_.Read(
        actual_queue_index_.index(), &context_.monotonic_sent_time,
        &context_.realtime_sent_time, &context_.size,
        reinterpret_cast<char *>(data_storage_.get()));
    if (read_result == ipc_lib::LocklessQueue::ReadResult::GOOD) {
      context_.queue_index = actual_queue_index_.index();
      data_ = reinterpret_cast<char *>(data_storage_.get()) +
              lockless_queue_.message_data_size() - context_.size;
      context_.data = data_;
      actual_queue_index_ = actual_queue_index_.Increment();
    }

    // Make sure the data wasn't modified while we were reading it.  This
    // can only happen if you are reading the last message *while* it is
    // being written to, which means you are pretty far behind.
    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::OVERWROTE)
        << ": Got behind while reading and the last message was modified "
           "out "
           "from under us while we were reading it.  Don't get so far "
           "behind.";

    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::TOO_OLD)
        << ": The next message is no longer available.";
    return read_result == ipc_lib::LocklessQueue::ReadResult::GOOD;
  }

  bool Fetch() override {
    const ipc_lib::QueueIndex queue_index = lockless_queue_.LatestQueueIndex();
    // actual_queue_index_ is only meaningful if it was set by Fetch or
    // FetchNext.  This happens when valid_data_ has been set.  So, only
    // skip checking if valid_data_ is true.
    //
    // Also, if the latest queue index is invalid, we are empty.  So there
    // is nothing to fetch.
    if ((data_ != nullptr &&
         queue_index == actual_queue_index_.DecrementBy(1u)) ||
        !queue_index.valid()) {
      return false;
    }

    ipc_lib::LocklessQueue::ReadResult read_result =
        lockless_queue_.Read(queue_index.index(), &context_.monotonic_sent_time,
                             &context_.realtime_sent_time, &context_.size,
                             reinterpret_cast<char *>(data_storage_.get()));
    if (read_result == ipc_lib::LocklessQueue::ReadResult::GOOD) {
      context_.queue_index = queue_index.index();
      data_ = reinterpret_cast<char *>(data_storage_.get()) +
              lockless_queue_.message_data_size() - context_.size;
      context_.data = data_;
      actual_queue_index_ = queue_index.Increment();
    }

    // Make sure the data wasn't modified while we were reading it.  This
    // can only happen if you are reading the last message *while* it is
    // being written to, which means you are pretty far behind.
    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::OVERWROTE)
        << ": Got behind while reading and the last message was modified "
           "out "
           "from under us while we were reading it.  Don't get so far "
           "behind.";

    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::NOTHING_NEW)
        << ": Queue index went backwards.  This should never happen.";

    // We fell behind between when we read the index and read the value.
    // This isn't worth recovering from since this means we went to sleep
    // for a long time in the middle of this function.
    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::TOO_OLD)
        << ": The next message is no longer available.";
    return read_result == ipc_lib::LocklessQueue::ReadResult::GOOD;
  }

  bool RegisterWakeup(int priority) {
    return lockless_queue_.RegisterWakeup(priority);
  }

  void UnregisterWakeup() { lockless_queue_.UnregisterWakeup(); }

 private:
  MMapedQueue lockless_queue_memory_;
  ipc_lib::LocklessQueue lockless_queue_;

  ipc_lib::QueueIndex actual_queue_index_ =
      ipc_lib::LocklessQueue::empty_queue_index();

  struct AlignedChar {
    alignas(32) char data;
  };

  std::unique_ptr<AlignedChar, decltype(&free)> data_storage_;
};

class ShmSender : public RawSender {
 public:
  explicit ShmSender(const Channel *channel, const ShmEventLoop *shm_event_loop)
      : RawSender(channel),
        shm_event_loop_(shm_event_loop),
        name_(channel->name()->str()),
        lockless_queue_memory_(channel),
        lockless_queue_(lockless_queue_memory_.memory(),
                        lockless_queue_memory_.config()),
        lockless_queue_sender_(lockless_queue_.MakeSender()) {}

  void *data() override { return lockless_queue_sender_.Data(); }
  size_t size() override { return lockless_queue_sender_.size(); }
  bool Send(size_t size) override {
    lockless_queue_sender_.Send(size);
    lockless_queue_.Wakeup(shm_event_loop_->priority());
    return true;
  }

  bool Send(const void *msg, size_t length) override {
    lockless_queue_sender_.Send(reinterpret_cast<const char *>(msg), length);
    lockless_queue_.Wakeup(shm_event_loop_->priority());
    // TODO(austin): Return an error if we send too fast.
    return true;
  }

  const std::string_view name() const override { return name_; }

 private:
  const ShmEventLoop *shm_event_loop_;
  std::string name_;
  MMapedQueue lockless_queue_memory_;
  ipc_lib::LocklessQueue lockless_queue_;
  ipc_lib::LocklessQueue::Sender lockless_queue_sender_;
};

}  // namespace

namespace internal {

// Class to manage the state for a Watcher.
class WatcherState {
 public:
  WatcherState(
      const Channel *channel,
      std::function<void(const Context &context, const void *message)> watcher)
      : shm_fetcher_(channel), watcher_(watcher) {}

  ~WatcherState() {}

  // Points the next message to fetch at the queue index which will be populated
  // next.
  void PointAtNextQueueIndex() { shm_fetcher_.PointAtNextQueueIndex(); }

  // Returns true if there is new data available.
  bool HasNewData() {
    if (!has_new_data_) {
      has_new_data_ = shm_fetcher_.FetchNext();
    }

    return has_new_data_;
  }

  // Returns the time of the current data sample.
  aos::monotonic_clock::time_point event_time() const {
    return shm_fetcher_.context().monotonic_sent_time;
  }

  // Consumes the data by calling the callback.
  void CallCallback() {
    CHECK(has_new_data_);
    watcher_(shm_fetcher_.context(), shm_fetcher_.most_recent_data());
    has_new_data_ = false;
  }

  // Starts the thread and waits until it is running.
  bool RegisterWakeup(int priority) {
    return shm_fetcher_.RegisterWakeup(priority);
  }

  void UnregisterWakeup() { return shm_fetcher_.UnregisterWakeup(); }

 private:
  bool has_new_data_ = false;

  ShmFetcher shm_fetcher_;

  std::function<void(const Context &context, const void *message)> watcher_;
};

// Adapter class to adapt a timerfd to a TimerHandler.
class TimerHandlerState : public TimerHandler {
 public:
  TimerHandlerState(ShmEventLoop *shm_event_loop, ::std::function<void()> fn)
      : shm_event_loop_(shm_event_loop), fn_(::std::move(fn)) {
    shm_event_loop_->epoll_.OnReadable(timerfd_.fd(), [this]() {
      const uint64_t elapsed_cycles = timerfd_.Read();

      shm_event_loop_->context_.monotonic_sent_time = base_;
      shm_event_loop_->context_.realtime_sent_time = realtime_clock::min_time;
      shm_event_loop_->context_.queue_index = 0;
      shm_event_loop_->context_.size = 0;
      shm_event_loop_->context_.data = nullptr;

      fn_();

      base_ += repeat_offset_ * elapsed_cycles;
    });
  }

  ~TimerHandlerState() { shm_event_loop_->epoll_.DeleteFd(timerfd_.fd()); }

  void Setup(monotonic_clock::time_point base,
             monotonic_clock::duration repeat_offset) override {
    timerfd_.SetTime(base, repeat_offset);
    base_ = base;
    repeat_offset_ = repeat_offset;
  }

  void Disable() override {
    // Disable is also threadsafe already.
    timerfd_.Disable();
  }

 private:
  ShmEventLoop *shm_event_loop_;

  TimerFd timerfd_;

  ::std::function<void()> fn_;

  monotonic_clock::time_point base_;
  monotonic_clock::duration repeat_offset_;
};

// Adapter class to the timerfd and PhasedLoop.
class PhasedLoopHandler : public ::aos::PhasedLoopHandler {
 public:
  PhasedLoopHandler(ShmEventLoop *shm_event_loop, ::std::function<void(int)> fn,
                    const monotonic_clock::duration interval,
                    const monotonic_clock::duration offset)
      : shm_event_loop_(shm_event_loop),
        phased_loop_(interval, shm_event_loop_->monotonic_now(), offset),
        fn_(::std::move(fn)) {
    shm_event_loop_->epoll_.OnReadable(timerfd_.fd(), [this]() {
      timerfd_.Read();
      // Update the context to hold the desired wakeup time.
      shm_event_loop_->context_.monotonic_sent_time = phased_loop_.sleep_time();
      shm_event_loop_->context_.realtime_sent_time = realtime_clock::min_time;
      shm_event_loop_->context_.queue_index = 0;
      shm_event_loop_->context_.size = 0;
      shm_event_loop_->context_.data = nullptr;

      // Compute how many cycles elapsed and schedule the next wakeup.
      Reschedule();

      // Call the function with the elapsed cycles.
      fn_(cycles_elapsed_);
      cycles_elapsed_ = 0;

      const monotonic_clock::time_point monotonic_end_time =
          monotonic_clock::now();

      // If the handler too too long so we blew by the previous deadline, we
      // want to just try for the next deadline.  Reschedule.
      if (monotonic_end_time > phased_loop_.sleep_time()) {
        Reschedule();
      }
    });
  }

  ~PhasedLoopHandler() { shm_event_loop_->epoll_.DeleteFd(timerfd_.fd()); }

  void set_interval_and_offset(
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset) override {
    phased_loop_.set_interval_and_offset(interval, offset);
  }

  void Startup() {
    phased_loop_.Reset(shm_event_loop_->monotonic_now());
    Reschedule();
    // The first time, we'll double count.  Reschedule here will count cycles
    // elapsed before now, and then the reschedule before runing the handler
    // will count the time that elapsed then.  So clear the count here.
    cycles_elapsed_ = 0;
  }

 private:
  // Reschedules the timer.
  void Reschedule() {
    cycles_elapsed_ += phased_loop_.Iterate(shm_event_loop_->monotonic_now());
    timerfd_.SetTime(phased_loop_.sleep_time(), ::aos::monotonic_clock::zero());
  }

  ShmEventLoop *shm_event_loop_;

  TimerFd timerfd_;
  time::PhasedLoop phased_loop_;

  int cycles_elapsed_ = 0;

  // Function to be run
  const ::std::function<void(int)> fn_;
};
}  // namespace internal

::std::unique_ptr<RawFetcher> ShmEventLoop::MakeRawFetcher(
    const Channel *channel) {
  ValidateChannel(channel);
  return ::std::unique_ptr<RawFetcher>(new ShmFetcher(channel));
}

::std::unique_ptr<RawSender> ShmEventLoop::MakeRawSender(
    const Channel *channel) {
  ValidateChannel(channel);
  Take(channel);
  return ::std::unique_ptr<RawSender>(new ShmSender(channel, this));
}

void ShmEventLoop::MakeRawWatcher(
    const Channel *channel,
    std::function<void(const Context &context, const void *message)> watcher) {
  ValidateChannel(channel);
  Take(channel);

  ::std::unique_ptr<internal::WatcherState> state(
      new internal::WatcherState(
      channel, std::move(watcher)));
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

void ShmEventLoop::Run() {
  std::unique_ptr<ipc_lib::SignalFd> signalfd;

  if (watchers_.size() > 0) {
    signalfd.reset(new ipc_lib::SignalFd({ipc_lib::kWakeupSignal}));

    epoll_.OnReadable(signalfd->fd(), [signalfd_ptr = signalfd.get(), this]() {
      signalfd_siginfo result = signalfd_ptr->Read();
      CHECK_EQ(result.ssi_signo, ipc_lib::kWakeupSignal);

      // TODO(austin): We should really be checking *everything*, not just
      // watchers, and calling the oldest thing first.  That will improve
      // determinism a lot.

      while (true) {
        // Call the handlers in time order of their messages.
        aos::monotonic_clock::time_point min_event_time =
            aos::monotonic_clock::max_time;
        size_t min_watcher_index = -1;
        size_t watcher_index = 0;
        for (::std::unique_ptr<internal::WatcherState> &watcher : watchers_) {
          if (watcher->HasNewData()) {
            if (watcher->event_time() < min_event_time) {
              min_watcher_index = watcher_index;
              min_event_time = watcher->event_time();
            }
          }
          ++watcher_index;
        }

        if (min_event_time == aos::monotonic_clock::max_time) {
          break;
        }

        watchers_[min_watcher_index]->CallCallback();
      }
    });
  }

  // Now, all the threads are up.  Lock everything into memory and go RT.
  if (priority_ != 0) {
    ::aos::InitRT();

    LOG(INFO) << "Setting priority to " << priority_;
    ::aos::SetCurrentThreadRealtimePriority(priority_);
  }

  set_is_running(true);

  // Now that we are realtime (but before the OnRun handlers run), snap the
  // queue index.
  for (::std::unique_ptr<internal::WatcherState> &watcher : watchers_) {
    watcher->PointAtNextQueueIndex();
    CHECK(watcher->RegisterWakeup(priority_));
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

  // And start our main event loop which runs all the timers and handles Quit.
  epoll_.Run();

  // Once epoll exits, there is no useful nonrt work left to do.
  set_is_running(false);

  // Nothing time or synchronization critical needs to happen after this point.
  // Drop RT priority.
  ::aos::UnsetCurrentThreadRealtimePriority();

  for (::std::unique_ptr<internal::WatcherState> &watcher : watchers_) {
    watcher->UnregisterWakeup();
  }

  if (watchers_.size() > 0) {
    epoll_.DeleteFd(signalfd->fd());
    signalfd.reset();
  }
}

void ShmEventLoop::Exit() { epoll_.Quit(); }

ShmEventLoop::~ShmEventLoop() {
  CHECK(!is_running()) << ": ShmEventLoop destroyed while running";
}

void ShmEventLoop::Take(const Channel *channel) {
  CHECK(!is_running()) << ": Cannot add new objects while running.";

  // Cheat aggresively.  Use the shared memory path as a proxy for a unique
  // identifier for the channel.
  const std::string path = ShmPath(channel);

  const auto prior = ::std::find(taken_.begin(), taken_.end(), path);
  CHECK(prior == taken_.end()) << ": " << path << " is already being used.";

  taken_.emplace_back(path);
}

void ShmEventLoop::SetRuntimeRealtimePriority(int priority) {
  if (is_running()) {
    LOG(FATAL) << "Cannot set realtime priority while running.";
  }
  priority_ = priority;
}

}  // namespace aos
