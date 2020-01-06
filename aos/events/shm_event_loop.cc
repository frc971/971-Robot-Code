#include "aos/events/shm_event_loop.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <iterator>
#include <stdexcept>

#include "aos/events/epoll.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/timing_statistics.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/ipc_lib/signalfd.h"
#include "aos/realtime.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/util/phased_loop.h"
#include "glog/logging.h"

namespace {

// Returns the portion of the path after the last /.  This very much assumes
// that the application name is null terminated.
const char *Filename(const char *path) {
  const std::string_view path_string_view = path;
  auto last_slash_pos = path_string_view.find_last_of("/");

  return last_slash_pos == std::string_view::npos ? path
                                                  : path + last_slash_pos + 1;
}

}  // namespace

DEFINE_string(shm_base, "/dev/shm/aos",
              "Directory to place queue backing mmaped files in.");
DEFINE_uint32(permissions, 0770,
              "Permissions to make shared memory files and folders.");
DEFINE_string(application_name, Filename(program_invocation_name),
              "The application name");

namespace aos {

void SetShmBase(const std::string_view base) {
  FLAGS_shm_base = std::string(base) + "/dev/shm/aos";
}

std::string ShmFolder(const Channel *channel) {
  CHECK(channel->has_name());
  CHECK_EQ(channel->name()->string_view()[0], '/');
  return FLAGS_shm_base + channel->name()->str() + "/";
}
std::string ShmPath(const Channel *channel) {
  CHECK(channel->has_type());
  return ShmFolder(channel) + channel->type()->str() + ".v1";
}

class MMapedQueue {
 public:
  MMapedQueue(const Channel *channel,
              const std::chrono::seconds channel_storage_duration) {
    std::string path = ShmPath(channel);

    config_.num_watchers = channel->num_watchers();
    config_.num_senders = channel->num_senders();
    config_.queue_size =
        channel_storage_duration.count() * channel->frequency();
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

  const ipc_lib::LocklessQueueConfiguration &config() const { return config_; }

 private:
  void MkdirP(std::string_view path) {
    auto last_slash_pos = path.find_last_of("/");

    std::string folder(last_slash_pos == std::string_view::npos
                           ? std::string_view("")
                           : path.substr(0, last_slash_pos));
    if (folder.empty()) return;
    MkdirP(folder);
    VLOG(1) << "Creating " << folder;
    const int result = mkdir(folder.c_str(), FLAGS_permissions);
    if (result == -1 && errno == EEXIST) {
      VLOG(1) << "Already exists";
      return;
    }
    PCHECK(result == 0) << ": Error creating " << folder;
  }

  ipc_lib::LocklessQueueConfiguration config_;

  int fd_;

  size_t size_;
  void *data_;
};

namespace {

const Node *MaybeMyNode(const Configuration *configuration) {
  if (!configuration->has_nodes()) {
    return nullptr;
  }

  return configuration::GetMyNode(configuration);
}

namespace chrono = ::std::chrono;

}  // namespace

ShmEventLoop::ShmEventLoop(const Configuration *configuration)
    : EventLoop(configuration),
      name_(FLAGS_application_name),
      node_(MaybeMyNode(configuration)) {
  if (configuration->has_nodes()) {
    CHECK(node_ != nullptr) << ": Couldn't find node in config.";
  }
}

namespace internal {

class SimpleShmFetcher {
 public:
  explicit SimpleShmFetcher(EventLoop *event_loop, const Channel *channel)
      : channel_(channel),
        lockless_queue_memory_(
            channel,
            chrono::ceil<chrono::seconds>(chrono::nanoseconds(
                event_loop->configuration()->channel_storage_duration()))),
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

  ~SimpleShmFetcher() {}

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

  bool FetchNext() {
    // TODO(austin): Get behind and make sure it dies both here and with
    // Fetch.
    ipc_lib::LocklessQueue::ReadResult read_result = lockless_queue_.Read(
        actual_queue_index_.index(), &context_.monotonic_event_time,
        &context_.realtime_event_time, &context_.monotonic_remote_time,
        &context_.realtime_remote_time, &context_.remote_queue_index,
        &context_.size, reinterpret_cast<char *>(data_storage_.get()));
    if (read_result == ipc_lib::LocklessQueue::ReadResult::GOOD) {
      context_.queue_index = actual_queue_index_.index();
      if (context_.remote_queue_index == 0xffffffffu) {
        context_.remote_queue_index = context_.queue_index;
      }
      if (context_.monotonic_remote_time == aos::monotonic_clock::min_time) {
        context_.monotonic_remote_time = context_.monotonic_event_time;
      }
      if (context_.realtime_remote_time == aos::realtime_clock::min_time) {
        context_.realtime_remote_time = context_.realtime_event_time;
      }
      context_.data = reinterpret_cast<char *>(data_storage_.get()) +
                      lockless_queue_.message_data_size() - context_.size;
      actual_queue_index_ = actual_queue_index_.Increment();
    }

    // Make sure the data wasn't modified while we were reading it.  This
    // can only happen if you are reading the last message *while* it is
    // being written to, which means you are pretty far behind.
    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::OVERWROTE)
        << ": Got behind while reading and the last message was modified "
           "out from under us while we were reading it.  Don't get so far "
           "behind.  "
        << configuration::CleanedChannelToString(channel_);

    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::TOO_OLD)
        << ": The next message is no longer available.  "
        << configuration::CleanedChannelToString(channel_);
    return read_result == ipc_lib::LocklessQueue::ReadResult::GOOD;
  }

  bool Fetch() {
    const ipc_lib::QueueIndex queue_index = lockless_queue_.LatestQueueIndex();
    // actual_queue_index_ is only meaningful if it was set by Fetch or
    // FetchNext.  This happens when valid_data_ has been set.  So, only
    // skip checking if valid_data_ is true.
    //
    // Also, if the latest queue index is invalid, we are empty.  So there
    // is nothing to fetch.
    if ((context_.data != nullptr &&
         queue_index == actual_queue_index_.DecrementBy(1u)) ||
        !queue_index.valid()) {
      return false;
    }

    ipc_lib::LocklessQueue::ReadResult read_result = lockless_queue_.Read(
        queue_index.index(), &context_.monotonic_event_time,
        &context_.realtime_event_time, &context_.monotonic_remote_time,
        &context_.realtime_remote_time, &context_.remote_queue_index,
        &context_.size, reinterpret_cast<char *>(data_storage_.get()));
    if (read_result == ipc_lib::LocklessQueue::ReadResult::GOOD) {
      context_.queue_index = queue_index.index();
      if (context_.remote_queue_index == 0xffffffffu) {
        context_.remote_queue_index = context_.queue_index;
      }
      if (context_.monotonic_remote_time == aos::monotonic_clock::min_time) {
        context_.monotonic_remote_time = context_.monotonic_event_time;
      }
      if (context_.realtime_remote_time == aos::realtime_clock::min_time) {
        context_.realtime_remote_time = context_.realtime_event_time;
      }
      context_.data = reinterpret_cast<char *>(data_storage_.get()) +
                      lockless_queue_.message_data_size() - context_.size;
      actual_queue_index_ = queue_index.Increment();
    }

    // Make sure the data wasn't modified while we were reading it.  This
    // can only happen if you are reading the last message *while* it is
    // being written to, which means you are pretty far behind.
    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::OVERWROTE)
        << ": Got behind while reading and the last message was modified "
           "out from under us while we were reading it.  Don't get so far "
           "behind."
        << configuration::CleanedChannelToString(channel_);

    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::NOTHING_NEW)
        << ": Queue index went backwards.  This should never happen.  "
        << configuration::CleanedChannelToString(channel_);

    // We fell behind between when we read the index and read the value.
    // This isn't worth recovering from since this means we went to sleep
    // for a long time in the middle of this function.
    CHECK(read_result != ipc_lib::LocklessQueue::ReadResult::TOO_OLD)
        << ": The next message is no longer available.  "
        << configuration::CleanedChannelToString(channel_);
    return read_result == ipc_lib::LocklessQueue::ReadResult::GOOD;
  }

  Context context() const { return context_; }

  bool RegisterWakeup(int priority) {
    return lockless_queue_.RegisterWakeup(priority);
  }

  void UnregisterWakeup() { lockless_queue_.UnregisterWakeup(); }

 private:
  const Channel *const channel_;
  MMapedQueue lockless_queue_memory_;
  ipc_lib::LocklessQueue lockless_queue_;

  ipc_lib::QueueIndex actual_queue_index_ =
      ipc_lib::LocklessQueue::empty_queue_index();

  struct AlignedChar {
    alignas(32) char data;
  };

  std::unique_ptr<AlignedChar, decltype(&free)> data_storage_;

  Context context_;
};

class ShmFetcher : public RawFetcher {
 public:
  explicit ShmFetcher(EventLoop *event_loop, const Channel *channel)
      : RawFetcher(event_loop, channel),
        simple_shm_fetcher_(event_loop, channel) {}

  ~ShmFetcher() { context_.data = nullptr; }

  std::pair<bool, monotonic_clock::time_point> DoFetchNext() override {
    if (simple_shm_fetcher_.FetchNext()) {
      context_ = simple_shm_fetcher_.context();
      return std::make_pair(true, monotonic_clock::now());
    }
    return std::make_pair(false, monotonic_clock::min_time);
  }

  std::pair<bool, monotonic_clock::time_point> DoFetch() override {
    if (simple_shm_fetcher_.Fetch()) {
      context_ = simple_shm_fetcher_.context();
      return std::make_pair(true, monotonic_clock::now());
    }
    return std::make_pair(false, monotonic_clock::min_time);
  }

 private:
  SimpleShmFetcher simple_shm_fetcher_;
};

class ShmSender : public RawSender {
 public:
  explicit ShmSender(EventLoop *event_loop, const Channel *channel)
      : RawSender(event_loop, channel),
        lockless_queue_memory_(
            channel,
            chrono::ceil<chrono::seconds>(chrono::nanoseconds(
                event_loop->configuration()->channel_storage_duration()))),
        lockless_queue_(lockless_queue_memory_.memory(),
                        lockless_queue_memory_.config()),
        lockless_queue_sender_(lockless_queue_.MakeSender()) {}

  ~ShmSender() override {}

  void *data() override { return lockless_queue_sender_.Data(); }
  size_t size() override { return lockless_queue_sender_.size(); }
  bool DoSend(size_t length,
              aos::monotonic_clock::time_point monotonic_remote_time,
              aos::realtime_clock::time_point realtime_remote_time,
              uint32_t remote_queue_index) override {
    lockless_queue_sender_.Send(
        length, monotonic_remote_time, realtime_remote_time, remote_queue_index,
        &monotonic_sent_time_, &realtime_sent_time_, &sent_queue_index_);
    lockless_queue_.Wakeup(event_loop()->priority());
    return true;
  }

  bool DoSend(const void *msg, size_t length,
              aos::monotonic_clock::time_point monotonic_remote_time,
              aos::realtime_clock::time_point realtime_remote_time,
              uint32_t remote_queue_index) override {
    lockless_queue_sender_.Send(reinterpret_cast<const char *>(msg), length,
                                monotonic_remote_time, realtime_remote_time,
                                remote_queue_index, &monotonic_sent_time_,
                                &realtime_sent_time_, &sent_queue_index_);
    lockless_queue_.Wakeup(event_loop()->priority());
    // TODO(austin): Return an error if we send too fast.
    return true;
  }

 private:
  MMapedQueue lockless_queue_memory_;
  ipc_lib::LocklessQueue lockless_queue_;
  ipc_lib::LocklessQueue::Sender lockless_queue_sender_;
};

// Class to manage the state for a Watcher.
class WatcherState : public aos::WatcherState {
 public:
  WatcherState(
      ShmEventLoop *event_loop, const Channel *channel,
      std::function<void(const Context &context, const void *message)> fn)
      : aos::WatcherState(event_loop, channel, std::move(fn)),
        event_loop_(event_loop),
        event_(this),
        simple_shm_fetcher_(event_loop, channel) {}

  ~WatcherState() override { event_loop_->RemoveEvent(&event_); }

  void Startup(EventLoop *event_loop) override {
    simple_shm_fetcher_.PointAtNextQueueIndex();
    CHECK(RegisterWakeup(event_loop->priority()));
  }

  // Returns true if there is new data available.
  bool CheckForNewData() {
    if (!has_new_data_) {
      has_new_data_ = simple_shm_fetcher_.FetchNext();

      if (has_new_data_) {
        event_.set_event_time(
            simple_shm_fetcher_.context().monotonic_event_time);
        event_loop_->AddEvent(&event_);
      }
    }

    return has_new_data_;
  }

  // Consumes the data by calling the callback.
  void HandleEvent() {
    CHECK(has_new_data_);
    DoCallCallback(monotonic_clock::now, simple_shm_fetcher_.context());
    has_new_data_ = false;
    CheckForNewData();
  }

  // Registers us to receive a signal on event reception.
  bool RegisterWakeup(int priority) {
    return simple_shm_fetcher_.RegisterWakeup(priority);
  }

  void UnregisterWakeup() { return simple_shm_fetcher_.UnregisterWakeup(); }

 private:
  bool has_new_data_ = false;

  ShmEventLoop *event_loop_;
  EventHandler<WatcherState> event_;
  SimpleShmFetcher simple_shm_fetcher_;
};

// Adapter class to adapt a timerfd to a TimerHandler.
class TimerHandlerState final : public TimerHandler {
 public:
  TimerHandlerState(ShmEventLoop *shm_event_loop, ::std::function<void()> fn)
      : TimerHandler(shm_event_loop, std::move(fn)),
        shm_event_loop_(shm_event_loop),
        event_(this) {
    shm_event_loop_->epoll_.OnReadable(
        timerfd_.fd(), [this]() { shm_event_loop_->HandleEvent(); });
  }

  ~TimerHandlerState() {
    Disable();
    shm_event_loop_->epoll_.DeleteFd(timerfd_.fd());
  }

  void HandleEvent() {
    uint64_t elapsed_cycles = timerfd_.Read();
    if (elapsed_cycles == 0u) {
      // We got called before the timer interrupt could happen, but because we
      // are checking the time, we got called on time.  Push the timer out by 1
      // cycle.
      elapsed_cycles = 1u;
      timerfd_.SetTime(base_ + repeat_offset_, repeat_offset_);
    }

    Call(monotonic_clock::now, base_);

    base_ += repeat_offset_ * elapsed_cycles;

    if (repeat_offset_ != chrono::seconds(0)) {
      event_.set_event_time(base_);
      shm_event_loop_->AddEvent(&event_);
    }
  }

  void Setup(monotonic_clock::time_point base,
             monotonic_clock::duration repeat_offset) override {
    if (event_.valid()) {
      shm_event_loop_->RemoveEvent(&event_);
    }

    timerfd_.SetTime(base, repeat_offset);
    base_ = base;
    repeat_offset_ = repeat_offset;
    event_.set_event_time(base_);
    shm_event_loop_->AddEvent(&event_);
  }

  void Disable() override {
    shm_event_loop_->RemoveEvent(&event_);
    timerfd_.Disable();
  }

 private:
  ShmEventLoop *shm_event_loop_;
  EventHandler<TimerHandlerState> event_;

  TimerFd timerfd_;

  monotonic_clock::time_point base_;
  monotonic_clock::duration repeat_offset_;
};

// Adapter class to the timerfd and PhasedLoop.
class PhasedLoopHandler final : public ::aos::PhasedLoopHandler {
 public:
  PhasedLoopHandler(ShmEventLoop *shm_event_loop, ::std::function<void(int)> fn,
                    const monotonic_clock::duration interval,
                    const monotonic_clock::duration offset)
      : aos::PhasedLoopHandler(shm_event_loop, std::move(fn), interval, offset),
        shm_event_loop_(shm_event_loop),
        event_(this) {
    shm_event_loop_->epoll_.OnReadable(
        timerfd_.fd(), [this]() { shm_event_loop_->HandleEvent(); });
  }

  void HandleEvent() {
    // The return value for read is the number of cycles that have elapsed.
    // Because we check to see when this event *should* have happened, there are
    // cases where Read() will return 0, when 1 cycle has actually happened.
    // This occurs when the timer interrupt hasn't triggered yet.  Therefore,
    // ignore it.  Call handles rescheduling and calculating elapsed cycles
    // without any extra help.
    timerfd_.Read();
    event_.Invalidate();

    Call(monotonic_clock::now, [this](monotonic_clock::time_point sleep_time) {
      Schedule(sleep_time);
    });
  }

  ~PhasedLoopHandler() override {
    shm_event_loop_->epoll_.DeleteFd(timerfd_.fd());
    shm_event_loop_->RemoveEvent(&event_);
  }

 private:
  // Reschedules the timer.
  void Schedule(monotonic_clock::time_point sleep_time) override {
    if (event_.valid()) {
      shm_event_loop_->RemoveEvent(&event_);
    }

    timerfd_.SetTime(sleep_time, ::aos::monotonic_clock::zero());
    event_.set_event_time(sleep_time);
    shm_event_loop_->AddEvent(&event_);
  }

  ShmEventLoop *shm_event_loop_;
  EventHandler<PhasedLoopHandler> event_;

  TimerFd timerfd_;
};
}  // namespace internal

::std::unique_ptr<RawFetcher> ShmEventLoop::MakeRawFetcher(
    const Channel *channel) {
  if (!configuration::ChannelIsReadableOnNode(channel, node())) {
    LOG(FATAL) << "Channel { \"name\": \"" << channel->name()->string_view()
               << "\", \"type\": \"" << channel->type()->string_view()
               << "\" } is not able to be fetched on this node.  Check your "
                  "configuration.";
  }

  return ::std::unique_ptr<RawFetcher>(new internal::ShmFetcher(this, channel));
}

::std::unique_ptr<RawSender> ShmEventLoop::MakeRawSender(
    const Channel *channel) {
  Take(channel);

  return ::std::unique_ptr<RawSender>(new internal::ShmSender(this, channel));
}

void ShmEventLoop::MakeRawWatcher(
    const Channel *channel,
    std::function<void(const Context &context, const void *message)> watcher) {
  Take(channel);

  if (!configuration::ChannelIsReadableOnNode(channel, node())) {
    LOG(FATAL) << "Channel { \"name\": \"" << channel->name()->string_view()
               << "\", \"type\": \"" << channel->type()->string_view()
               << "\" } is not able to be watched on this node.  Check your "
                  "configuration.";
  }

  NewWatcher(::std::unique_ptr<WatcherState>(
      new internal::WatcherState(this, channel, std::move(watcher))));
}

TimerHandler *ShmEventLoop::AddTimer(::std::function<void()> callback) {
  return NewTimer(::std::unique_ptr<TimerHandler>(
      new internal::TimerHandlerState(this, ::std::move(callback))));
}

PhasedLoopHandler *ShmEventLoop::AddPhasedLoop(
    ::std::function<void(int)> callback,
    const monotonic_clock::duration interval,
    const monotonic_clock::duration offset) {
  return NewPhasedLoop(
      ::std::unique_ptr<PhasedLoopHandler>(new internal::PhasedLoopHandler(
          this, ::std::move(callback), interval, offset)));
}

void ShmEventLoop::OnRun(::std::function<void()> on_run) {
  on_run_.push_back(::std::move(on_run));
}

void ShmEventLoop::HandleEvent() {
  // Update all the times for handlers.
  for (::std::unique_ptr<WatcherState> &base_watcher : watchers_) {
    internal::WatcherState *watcher =
        reinterpret_cast<internal::WatcherState *>(base_watcher.get());

    watcher->CheckForNewData();
  }

  while (true) {
    if (EventCount() == 0 ||
        PeekEvent()->event_time() > monotonic_clock::now()) {
      break;
    }

    EventLoopEvent *event = PopEvent();
    event->HandleEvent();
  }
}

// RAII class to mask signals.
class ScopedSignalMask {
 public:
  ScopedSignalMask(std::initializer_list<int> signals) {
    sigset_t sigset;
    PCHECK(sigemptyset(&sigset) == 0);
    for (int signal : signals) {
      PCHECK(sigaddset(&sigset, signal) == 0);
    }

    PCHECK(sigprocmask(SIG_BLOCK, &sigset, &old_) == 0);
  }

  ~ScopedSignalMask() { PCHECK(sigprocmask(SIG_SETMASK, &old_, nullptr) == 0); }

 private:
  sigset_t old_;
};

// Class to manage the static state associated with killing multiple event
// loops.
class SignalHandler {
 public:
  // Gets the singleton.
  static SignalHandler *global() {
    static SignalHandler loop;
    return &loop;
  }

  // Handles the signal with the singleton.
  static void HandleSignal(int) { global()->DoHandleSignal(); }

  // Registers an event loop to receive Exit() calls.
  void Register(ShmEventLoop *event_loop) {
    // Block signals while we have the mutex so we never race with the signal
    // handler.
    ScopedSignalMask mask({SIGINT, SIGHUP, SIGTERM});
    std::unique_lock<stl_mutex> locker(mutex_);
    if (event_loops_.size() == 0) {
      // The first caller registers the signal handler.
      struct sigaction new_action;
      sigemptyset(&new_action.sa_mask);
      // This makes it so that 2 control c's to a stuck process will kill it by
      // restoring the original signal handler.
      new_action.sa_flags = SA_RESETHAND;
      new_action.sa_handler = &HandleSignal;

      PCHECK(sigaction(SIGINT, &new_action, &old_action_int_) == 0);
      PCHECK(sigaction(SIGHUP, &new_action, &old_action_hup_) == 0);
      PCHECK(sigaction(SIGTERM, &new_action, &old_action_term_) == 0);
    }

    event_loops_.push_back(event_loop);
  }

  // Unregisters an event loop to receive Exit() calls.
  void Unregister(ShmEventLoop *event_loop) {
    // Block signals while we have the mutex so we never race with the signal
    // handler.
    ScopedSignalMask mask({SIGINT, SIGHUP, SIGTERM});
    std::unique_lock<stl_mutex> locker(mutex_);

    event_loops_.erase(std::find(event_loops_.begin(), event_loops_.end(), event_loop));

    if (event_loops_.size() == 0u) {
      // The last caller restores the original signal handlers.
      PCHECK(sigaction(SIGINT, &old_action_int_, nullptr) == 0);
      PCHECK(sigaction(SIGHUP, &old_action_hup_, nullptr) == 0);
      PCHECK(sigaction(SIGTERM, &old_action_term_, nullptr) == 0);
    }
  }

 private:
  void DoHandleSignal() {
    // We block signals while grabbing the lock, so there should never be a
    // race.  Confirm that this is true using trylock.
    CHECK(mutex_.try_lock()) << ": sigprocmask failed to block signals while "
                                "modifing the event loop list.";
    for (ShmEventLoop *event_loop : event_loops_) {
      event_loop->Exit();
    }
    mutex_.unlock();
  }

  // Mutex to protect all state.
  stl_mutex mutex_;
  std::vector<ShmEventLoop *> event_loops_;
  struct sigaction old_action_int_;
  struct sigaction old_action_hup_;
  struct sigaction old_action_term_;
};

void ShmEventLoop::Run() {
  SignalHandler::global()->Register(this);

  std::unique_ptr<ipc_lib::SignalFd> signalfd;

  if (watchers_.size() > 0) {
    signalfd.reset(new ipc_lib::SignalFd({ipc_lib::kWakeupSignal}));

    epoll_.OnReadable(signalfd->fd(), [signalfd_ptr = signalfd.get(), this]() {
      signalfd_siginfo result = signalfd_ptr->Read();
      CHECK_EQ(result.ssi_signo, ipc_lib::kWakeupSignal);

      // TODO(austin): We should really be checking *everything*, not just
      // watchers, and calling the oldest thing first.  That will improve
      // determinism a lot.

      HandleEvent();
    });
  }

  MaybeScheduleTimingReports();

  ReserveEvents();

  // Now, all the callbacks are setup.  Lock everything into memory and go RT.
  if (priority_ != 0) {
    ::aos::InitRT();

    LOG(INFO) << "Setting priority to " << priority_;
    ::aos::SetCurrentThreadRealtimePriority(priority_);
  }

  set_is_running(true);

  // Now that we are realtime (but before the OnRun handlers run), snap the
  // queue index.
  for (::std::unique_ptr<WatcherState> &watcher : watchers_) {
    watcher->Startup(this);
  }

  // Now that we are RT, run all the OnRun handlers.
  for (const auto &run : on_run_) {
    run();
  }

  // And start our main event loop which runs all the timers and handles Quit.
  epoll_.Run();

  // Once epoll exits, there is no useful nonrt work left to do.
  set_is_running(false);

  // Nothing time or synchronization critical needs to happen after this point.
  // Drop RT priority.
  ::aos::UnsetCurrentThreadRealtimePriority();

  for (::std::unique_ptr<WatcherState> &base_watcher : watchers_) {
    internal::WatcherState *watcher =
        reinterpret_cast<internal::WatcherState *>(base_watcher.get());
    watcher->UnregisterWakeup();
  }

  if (watchers_.size() > 0) {
    epoll_.DeleteFd(signalfd->fd());
    signalfd.reset();
  }

  SignalHandler::global()->Unregister(this);

  // Trigger any remaining senders or fetchers to be cleared before destroying
  // the event loop so the book keeping matches.  Do this in the thread that
  // created the timing reporter.
  timing_report_sender_.reset();
}

void ShmEventLoop::Exit() { epoll_.Quit(); }

ShmEventLoop::~ShmEventLoop() {
  // Force everything with a registered fd with epoll to be destroyed now.
  timers_.clear();
  phased_loops_.clear();
  watchers_.clear();

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

pid_t ShmEventLoop::GetTid() { return syscall(SYS_gettid); }

}  // namespace aos
