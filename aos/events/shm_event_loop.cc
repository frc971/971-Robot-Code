#include "aos/events/shm_event_loop.h"

#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iterator>
#include <stdexcept>

#include "glog/logging.h"

#include "aos/events/aos_logging.h"
#include "aos/events/epoll.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/timing_statistics.h"
#include "aos/init.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/ipc_lib/memory_mapped_queue.h"
#include "aos/realtime.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/util/file.h"
#include "aos/util/phased_loop.h"

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
// This value is affected by the umask of the process which is calling it
// and is set to the user's value by default (check yours running `umask` on
// the command line).
// Any file mode requested is transformed using: mode & ~umask and the default
// umask is 0022 (allow any permissions for the user, dont allow writes for
// groups or others).
// See https://man7.org/linux/man-pages/man2/umask.2.html for more details.
// WITH THE DEFAULT UMASK YOU WONT ACTUALLY GET THESE PERMISSIONS :)
DEFINE_uint32(permissions, 0770,
              "Permissions to make shared memory files and folders, "
              "affected by the process's umask. "
              "See shm_event_loop.cc for more details.");
DEFINE_string(application_name, Filename(program_invocation_name),
              "The application name");

namespace aos {

using namespace shm_event_loop_internal;

void SetShmBase(const std::string_view base) {
  FLAGS_shm_base = std::string(base) + "/aos";
}

namespace {

const Node *MaybeMyNode(const Configuration *configuration) {
  if (!configuration->has_nodes()) {
    return nullptr;
  }

  return configuration::GetMyNode(configuration);
}

}  // namespace

ShmEventLoop::ShmEventLoop(const Configuration *configuration)
    : EventLoop(configuration),
      boot_uuid_(UUID::BootUUID()),
      shm_base_(FLAGS_shm_base),
      name_(FLAGS_application_name),
      node_(MaybeMyNode(configuration)) {
  CHECK(IsInitialized()) << ": Need to initialize AOS first.";
  ClearContext();
  if (configuration->has_nodes()) {
    CHECK(node_ != nullptr) << ": Couldn't find node in config.";
  }
}

namespace shm_event_loop_internal {

class SimpleShmFetcher {
 public:
  explicit SimpleShmFetcher(std::string_view shm_base, ShmEventLoop *event_loop,
                            const Channel *channel)
      : event_loop_(event_loop),
        channel_(channel),
        lockless_queue_memory_(shm_base, FLAGS_permissions,
                               event_loop->configuration(), channel),
        reader_(lockless_queue_memory_.queue()) {
    context_.data = nullptr;
    // Point the queue index at the next index to read starting now.  This
    // makes it such that FetchNext will read the next message sent after
    // the fetcher is created.
    PointAtNextQueueIndex();
  }

  ~SimpleShmFetcher() {}

  // Sets this object to pin or copy data, as configured in the channel.
  void RetrieveData() {
    if (channel_->read_method() == ReadMethod::PIN) {
      PinDataOnFetch();
    } else {
      CopyDataOnFetch();
    }
  }

  // Sets this object to copy data out of the shared memory into a private
  // buffer when fetching.
  void CopyDataOnFetch() {
    CHECK(!pin_data());
    data_storage_.reset(static_cast<char *>(
        malloc(channel_->max_size() + kChannelDataAlignment - 1)));
  }

  // Sets this object to pin data in shared memory when fetching.
  void PinDataOnFetch() {
    CHECK(!copy_data());
    auto maybe_pinner =
        ipc_lib::LocklessQueuePinner::Make(lockless_queue_memory_.queue());
    if (!maybe_pinner) {
      LOG(FATAL) << "Failed to create reader on "
                 << configuration::CleanedChannelToString(channel_)
                 << ", too many readers.";
    }
    pinner_ = std::move(maybe_pinner.value());
  }

  // Points the next message to fetch at the queue index which will be
  // populated next.
  void PointAtNextQueueIndex() {
    actual_queue_index_ = reader_.LatestIndex();
    if (!actual_queue_index_.valid()) {
      // Nothing in the queue.  The next element will show up at the 0th
      // index in the queue.
      actual_queue_index_ = ipc_lib::QueueIndex::Zero(
          LocklessQueueSize(lockless_queue_memory_.memory()));
    } else {
      actual_queue_index_ = actual_queue_index_.Increment();
    }
  }

  bool FetchNext() {
    const ipc_lib::LocklessQueueReader::Result read_result =
        DoFetch(actual_queue_index_);

    return read_result == ipc_lib::LocklessQueueReader::Result::GOOD;
  }

  bool Fetch() {
    const ipc_lib::QueueIndex queue_index = reader_.LatestIndex();
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

    const ipc_lib::LocklessQueueReader::Result read_result =
        DoFetch(queue_index);

    CHECK(read_result != ipc_lib::LocklessQueueReader::Result::NOTHING_NEW)
        << ": Queue index went backwards.  This should never happen.  "
        << configuration::CleanedChannelToString(channel_);

    return read_result == ipc_lib::LocklessQueueReader::Result::GOOD;
  }

  Context context() const { return context_; }

  bool RegisterWakeup(int priority) {
    CHECK(!watcher_);
    watcher_ = ipc_lib::LocklessQueueWatcher::Make(
        lockless_queue_memory_.queue(), priority);
    return static_cast<bool>(watcher_);
  }

  void UnregisterWakeup() {
    CHECK(watcher_);
    watcher_ = std::nullopt;
  }

  absl::Span<char> GetMutableSharedMemory() {
    return lockless_queue_memory_.GetMutableSharedMemory();
  }

  absl::Span<const char> GetConstSharedMemory() const {
    return lockless_queue_memory_.GetConstSharedMemory();
  }

  absl::Span<const char> GetPrivateMemory() const {
    if (pin_data()) {
      return lockless_queue_memory_.GetConstSharedMemory();
    }
    return absl::Span<char>(
        const_cast<SimpleShmFetcher *>(this)->data_storage_start(),
        LocklessQueueMessageDataSize(lockless_queue_memory_.memory()));
  }

 private:
  ipc_lib::LocklessQueueReader::Result DoFetch(
      ipc_lib::QueueIndex queue_index) {
    // TODO(austin): Get behind and make sure it dies.
    char *copy_buffer = nullptr;
    if (copy_data()) {
      copy_buffer = data_storage_start();
    }
    ipc_lib::LocklessQueueReader::Result read_result = reader_.Read(
        queue_index.index(), &context_.monotonic_event_time,
        &context_.realtime_event_time, &context_.monotonic_remote_time,
        &context_.realtime_remote_time, &context_.remote_queue_index,
        &context_.source_boot_uuid, &context_.size, copy_buffer);

    if (read_result == ipc_lib::LocklessQueueReader::Result::GOOD) {
      if (pin_data()) {
        const int pin_result = pinner_->PinIndex(queue_index.index());
        CHECK(pin_result >= 0)
            << ": Got behind while reading and the last message was modified "
               "out from under us while we tried to pin it. Don't get so far "
               "behind on: "
            << configuration::CleanedChannelToString(channel_);
        context_.buffer_index = pin_result;
      } else {
        context_.buffer_index = -1;
      }

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
      const char *const data = DataBuffer();
      if (data) {
        context_.data =
            data +
            LocklessQueueMessageDataSize(lockless_queue_memory_.memory()) -
            context_.size;
      } else {
        context_.data = nullptr;
      }
      actual_queue_index_ = queue_index.Increment();
    }

    // Make sure the data wasn't modified while we were reading it.  This
    // can only happen if you are reading the last message *while* it is
    // being written to, which means you are pretty far behind.
    CHECK(read_result != ipc_lib::LocklessQueueReader::Result::OVERWROTE)
        << ": Got behind while reading and the last message was modified "
           "out from under us while we were reading it.  Don't get so far "
           "behind on: "
        << configuration::CleanedChannelToString(channel_);

    // We fell behind between when we read the index and read the value.
    // This isn't worth recovering from since this means we went to sleep
    // for a long time in the middle of this function.
    if (read_result == ipc_lib::LocklessQueueReader::Result::TOO_OLD) {
      event_loop_->SendTimingReport();
      LOG(FATAL) << "The next message is no longer available.  "
                 << configuration::CleanedChannelToString(channel_);
    }

    return read_result;
  }

  char *data_storage_start() const {
    CHECK(copy_data());
    return RoundChannelData(data_storage_.get(), channel_->max_size());
  }

  // Note that for some modes the return value will change as new messages are
  // read.
  const char *DataBuffer() const {
    if (copy_data()) {
      return data_storage_start();
    }
    if (pin_data()) {
      return static_cast<const char *>(pinner_->Data());
    }
    return nullptr;
  }

  bool copy_data() const { return static_cast<bool>(data_storage_); }
  bool pin_data() const { return static_cast<bool>(pinner_); }

  aos::ShmEventLoop *event_loop_;
  const Channel *const channel_;
  ipc_lib::MemoryMappedQueue lockless_queue_memory_;
  ipc_lib::LocklessQueueReader reader_;
  // This being nullopt indicates we're not looking for wakeups right now.
  std::optional<ipc_lib::LocklessQueueWatcher> watcher_;

  ipc_lib::QueueIndex actual_queue_index_ = ipc_lib::QueueIndex::Invalid();

  // This being empty indicates we're not going to copy data.
  std::unique_ptr<char, decltype(&free)> data_storage_{nullptr, &free};

  // This being nullopt indicates we're not going to pin messages.
  std::optional<ipc_lib::LocklessQueuePinner> pinner_;

  Context context_;
};

class ShmFetcher : public RawFetcher {
 public:
  explicit ShmFetcher(std::string_view shm_base, ShmEventLoop *event_loop,
                      const Channel *channel)
      : RawFetcher(event_loop, channel),
        simple_shm_fetcher_(shm_base, event_loop, channel) {
    simple_shm_fetcher_.RetrieveData();
  }

  ~ShmFetcher() override {
    shm_event_loop()->CheckCurrentThread();
    context_.data = nullptr;
  }

  std::pair<bool, monotonic_clock::time_point> DoFetchNext() override {
    shm_event_loop()->CheckCurrentThread();
    if (simple_shm_fetcher_.FetchNext()) {
      context_ = simple_shm_fetcher_.context();
      return std::make_pair(true, monotonic_clock::now());
    }
    return std::make_pair(false, monotonic_clock::min_time);
  }

  std::pair<bool, monotonic_clock::time_point> DoFetch() override {
    shm_event_loop()->CheckCurrentThread();
    if (simple_shm_fetcher_.Fetch()) {
      context_ = simple_shm_fetcher_.context();
      return std::make_pair(true, monotonic_clock::now());
    }
    return std::make_pair(false, monotonic_clock::min_time);
  }

  absl::Span<const char> GetPrivateMemory() const {
    return simple_shm_fetcher_.GetPrivateMemory();
  }

 private:
  const ShmEventLoop *shm_event_loop() const {
    return static_cast<const ShmEventLoop *>(event_loop());
  }

  SimpleShmFetcher simple_shm_fetcher_;
};

class ShmExitHandle : public ExitHandle {
 public:
  ShmExitHandle(ShmEventLoop *event_loop) : event_loop_(event_loop) {
    ++event_loop_->exit_handle_count_;
  }
  ~ShmExitHandle() override {
    CHECK_GT(event_loop_->exit_handle_count_, 0);
    --event_loop_->exit_handle_count_;
  }

  void Exit() override { event_loop_->Exit(); }

 private:
  ShmEventLoop *const event_loop_;
};

class ShmSender : public RawSender {
 public:
  explicit ShmSender(std::string_view shm_base, EventLoop *event_loop,
                     const Channel *channel)
      : RawSender(event_loop, channel),
        lockless_queue_memory_(shm_base, FLAGS_permissions,
                               event_loop->configuration(), channel),
        lockless_queue_sender_(
            VerifySender(ipc_lib::LocklessQueueSender::Make(
                             lockless_queue_memory_.queue(),
                             configuration::ChannelStorageDuration(
                                 event_loop->configuration(), channel)),
                         channel)),
        wake_upper_(lockless_queue_memory_.queue()) {}

  ~ShmSender() override { shm_event_loop()->CheckCurrentThread(); }

  static ipc_lib::LocklessQueueSender VerifySender(
      std::optional<ipc_lib::LocklessQueueSender> sender,
      const Channel *channel) {
    if (sender) {
      return std::move(sender.value());
    }
    LOG(FATAL) << "Failed to create sender on "
               << configuration::CleanedChannelToString(channel)
               << ", too many senders.";
  }

  void *data() override {
    shm_event_loop()->CheckCurrentThread();
    return lockless_queue_sender_.Data();
  }
  size_t size() override {
    shm_event_loop()->CheckCurrentThread();
    return lockless_queue_sender_.size();
  }

  Error DoSend(size_t length,
               aos::monotonic_clock::time_point monotonic_remote_time,
               aos::realtime_clock::time_point realtime_remote_time,
               uint32_t remote_queue_index,
               const UUID &source_boot_uuid) override {
    shm_event_loop()->CheckCurrentThread();
    CHECK_LE(length, static_cast<size_t>(channel()->max_size()))
        << ": Sent too big a message on "
        << configuration::CleanedChannelToString(channel());
    const auto result = lockless_queue_sender_.Send(
        length, monotonic_remote_time, realtime_remote_time, remote_queue_index,
        source_boot_uuid, &monotonic_sent_time_, &realtime_sent_time_,
        &sent_queue_index_);
    CHECK_NE(result, ipc_lib::LocklessQueueSender::Result::INVALID_REDZONE)
        << ": Somebody wrote outside the buffer of their message on channel "
        << configuration::CleanedChannelToString(channel());

    wake_upper_.Wakeup(event_loop()->is_running()
                           ? event_loop()->runtime_realtime_priority()
                           : 0);
    return CheckLocklessQueueResult(result);
  }

  Error DoSend(const void *msg, size_t length,
               aos::monotonic_clock::time_point monotonic_remote_time,
               aos::realtime_clock::time_point realtime_remote_time,
               uint32_t remote_queue_index,
               const UUID &source_boot_uuid) override {
    shm_event_loop()->CheckCurrentThread();
    CHECK_LE(length, static_cast<size_t>(channel()->max_size()))
        << ": Sent too big a message on "
        << configuration::CleanedChannelToString(channel());
    const auto result = lockless_queue_sender_.Send(
        reinterpret_cast<const char *>(msg), length, monotonic_remote_time,
        realtime_remote_time, remote_queue_index, source_boot_uuid,
        &monotonic_sent_time_, &realtime_sent_time_, &sent_queue_index_);

    CHECK_NE(result, ipc_lib::LocklessQueueSender::Result::INVALID_REDZONE)
        << ": Somebody wrote outside the buffer of their message on "
           "channel "
        << configuration::CleanedChannelToString(channel());
    wake_upper_.Wakeup(event_loop()->is_running()
                           ? event_loop()->runtime_realtime_priority()
                           : 0);

    return CheckLocklessQueueResult(result);
  }

  absl::Span<char> GetSharedMemory() const {
    return lockless_queue_memory_.GetMutableSharedMemory();
  }

  int buffer_index() override {
    shm_event_loop()->CheckCurrentThread();
    return lockless_queue_sender_.buffer_index();
  }

 private:
  const ShmEventLoop *shm_event_loop() const {
    return static_cast<const ShmEventLoop *>(event_loop());
  }

  RawSender::Error CheckLocklessQueueResult(
      const ipc_lib::LocklessQueueSender::Result &result) {
    switch (result) {
      case ipc_lib::LocklessQueueSender::Result::GOOD:
        return Error::kOk;
      case ipc_lib::LocklessQueueSender::Result::MESSAGES_SENT_TOO_FAST:
        return Error::kMessagesSentTooFast;
      case ipc_lib::LocklessQueueSender::Result::INVALID_REDZONE:
        return Error::kInvalidRedzone;
    }
    LOG(FATAL) << "Unknown lockless queue sender result"
               << static_cast<int>(result);
  }

  ipc_lib::MemoryMappedQueue lockless_queue_memory_;
  ipc_lib::LocklessQueueSender lockless_queue_sender_;
  ipc_lib::LocklessQueueWakeUpper wake_upper_;
};

// Class to manage the state for a Watcher.
class ShmWatcherState : public WatcherState {
 public:
  ShmWatcherState(
      std::string_view shm_base, ShmEventLoop *event_loop,
      const Channel *channel,
      std::function<void(const Context &context, const void *message)> fn,
      bool copy_data)
      : WatcherState(event_loop, channel, std::move(fn)),
        event_loop_(event_loop),
        event_(this),
        simple_shm_fetcher_(shm_base, event_loop, channel) {
    if (copy_data) {
      simple_shm_fetcher_.RetrieveData();
    }
  }

  ~ShmWatcherState() override {
    event_loop_->CheckCurrentThread();
    event_loop_->RemoveEvent(&event_);
  }

  void Startup(EventLoop *event_loop) override {
    event_loop_->CheckCurrentThread();
    simple_shm_fetcher_.PointAtNextQueueIndex();
    CHECK(RegisterWakeup(event_loop->runtime_realtime_priority()));
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

  absl::Span<const char> GetSharedMemory() const {
    return simple_shm_fetcher_.GetConstSharedMemory();
  }

 private:
  bool has_new_data_ = false;

  ShmEventLoop *event_loop_;
  EventHandler<ShmWatcherState> event_;
  SimpleShmFetcher simple_shm_fetcher_;
};

// Adapter class to adapt a timerfd to a TimerHandler.
class ShmTimerHandler final : public TimerHandler {
 public:
  ShmTimerHandler(ShmEventLoop *shm_event_loop, ::std::function<void()> fn)
      : TimerHandler(shm_event_loop, std::move(fn)),
        shm_event_loop_(shm_event_loop),
        event_(this) {
    shm_event_loop_->epoll_.OnReadable(timerfd_.fd(), [this]() {
      // The timer may fire spuriously.  HandleEvent on the event loop will
      // call the callback if it is needed.  It may also have called it when
      // processing some other event, and the kernel decided to deliver this
      // wakeup anyways.
      timerfd_.Read();
      shm_event_loop_->HandleEvent();
    });
  }

  ~ShmTimerHandler() {
    shm_event_loop_->CheckCurrentThread();
    Disable();
    shm_event_loop_->epoll_.DeleteFd(timerfd_.fd());
  }

  void HandleEvent() {
    CHECK(!event_.valid());
    disabled_ = false;
    const auto monotonic_now = Call(monotonic_clock::now, base_);
    if (event_.valid()) {
      // If someone called Schedule inside Call, rescheduling is already taken
      // care of. Bail.
      return;
    }
    if (disabled_) {
      // Somebody called Disable inside Call, so we don't want to reschedule.
      // Bail.
      return;
    }

    if (repeat_offset_ == std::chrono::seconds(0)) {
      timerfd_.Disable();
      disabled_ = true;
    } else {
      // Compute how many cycles have elapsed and schedule the next iteration
      // for the next iteration in the future.
      const int elapsed_cycles =
          std::max<int>(0, (monotonic_now - base_ + repeat_offset_ -
                            std::chrono::nanoseconds(1)) /
                               repeat_offset_);
      base_ += repeat_offset_ * elapsed_cycles;

      // Update the heap and schedule the timerfd wakeup.
      event_.set_event_time(base_);
      shm_event_loop_->AddEvent(&event_);
      timerfd_.SetTime(base_, std::chrono::seconds(0));
      disabled_ = false;
    }
  }

  void Schedule(monotonic_clock::time_point base,
                monotonic_clock::duration repeat_offset) override {
    shm_event_loop_->CheckCurrentThread();
    if (event_.valid()) {
      shm_event_loop_->RemoveEvent(&event_);
    }

    timerfd_.SetTime(base, repeat_offset);
    base_ = base;
    repeat_offset_ = repeat_offset;
    event_.set_event_time(base_);
    shm_event_loop_->AddEvent(&event_);
    disabled_ = false;
  }

  void Disable() override {
    shm_event_loop_->CheckCurrentThread();
    shm_event_loop_->RemoveEvent(&event_);
    timerfd_.Disable();
    disabled_ = true;
  }

  bool IsDisabled() override { return disabled_; }

 private:
  ShmEventLoop *shm_event_loop_;
  EventHandler<ShmTimerHandler> event_;

  internal::TimerFd timerfd_;

  monotonic_clock::time_point base_;
  monotonic_clock::duration repeat_offset_;

  // Used to track if Disable() was called during the callback, so we know not
  // to reschedule.
  bool disabled_ = true;
};

// Adapter class to the timerfd and PhasedLoop.
class ShmPhasedLoopHandler final : public PhasedLoopHandler {
 public:
  ShmPhasedLoopHandler(ShmEventLoop *shm_event_loop,
                       ::std::function<void(int)> fn,
                       const monotonic_clock::duration interval,
                       const monotonic_clock::duration offset)
      : PhasedLoopHandler(shm_event_loop, std::move(fn), interval, offset),
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

    Call(monotonic_clock::now);
  }

  ~ShmPhasedLoopHandler() override {
    shm_event_loop_->CheckCurrentThread();
    shm_event_loop_->epoll_.DeleteFd(timerfd_.fd());
    shm_event_loop_->RemoveEvent(&event_);
  }

 private:
  // Reschedules the timer.
  void Schedule(monotonic_clock::time_point sleep_time) override {
    shm_event_loop_->CheckCurrentThread();
    if (event_.valid()) {
      shm_event_loop_->RemoveEvent(&event_);
    }

    timerfd_.SetTime(sleep_time, ::aos::monotonic_clock::zero());
    event_.set_event_time(sleep_time);
    shm_event_loop_->AddEvent(&event_);
  }

  ShmEventLoop *shm_event_loop_;
  EventHandler<ShmPhasedLoopHandler> event_;

  internal::TimerFd timerfd_;
};

}  // namespace shm_event_loop_internal

::std::unique_ptr<RawFetcher> ShmEventLoop::MakeRawFetcher(
    const Channel *channel) {
  CheckCurrentThread();
  if (!configuration::ChannelIsReadableOnNode(channel, node())) {
    LOG(FATAL) << "Channel { \"name\": \"" << channel->name()->string_view()
               << "\", \"type\": \"" << channel->type()->string_view()
               << "\" } is not able to be fetched on this node.  Check your "
                  "configuration.";
  }

  return ::std::unique_ptr<RawFetcher>(
      new ShmFetcher(shm_base_, this, channel));
}

::std::unique_ptr<RawSender> ShmEventLoop::MakeRawSender(
    const Channel *channel) {
  CheckCurrentThread();
  TakeSender(channel);

  return ::std::unique_ptr<RawSender>(new ShmSender(shm_base_, this, channel));
}

void ShmEventLoop::MakeRawWatcher(
    const Channel *channel,
    std::function<void(const Context &context, const void *message)> watcher) {
  CheckCurrentThread();
  TakeWatcher(channel);

  NewWatcher(::std::unique_ptr<WatcherState>(
      new ShmWatcherState(shm_base_, this, channel, std::move(watcher), true)));
}

void ShmEventLoop::MakeRawNoArgWatcher(
    const Channel *channel,
    std::function<void(const Context &context)> watcher) {
  CheckCurrentThread();
  TakeWatcher(channel);

  NewWatcher(::std::unique_ptr<WatcherState>(new ShmWatcherState(
      shm_base_, this, channel,
      [watcher](const Context &context, const void *) { watcher(context); },
      false)));
}

TimerHandler *ShmEventLoop::AddTimer(::std::function<void()> callback) {
  CheckCurrentThread();
  return NewTimer(::std::unique_ptr<TimerHandler>(
      new ShmTimerHandler(this, ::std::move(callback))));
}

PhasedLoopHandler *ShmEventLoop::AddPhasedLoop(
    ::std::function<void(int)> callback,
    const monotonic_clock::duration interval,
    const monotonic_clock::duration offset) {
  CheckCurrentThread();
  return NewPhasedLoop(::std::unique_ptr<PhasedLoopHandler>(
      new ShmPhasedLoopHandler(this, ::std::move(callback), interval, offset)));
}

void ShmEventLoop::OnRun(::std::function<void()> on_run) {
  CheckCurrentThread();
  on_run_.push_back(::std::move(on_run));
}

void ShmEventLoop::CheckCurrentThread() const {
  if (__builtin_expect(check_mutex_ != nullptr, false)) {
    CHECK(check_mutex_->is_locked())
        << ": The configured mutex is not locked while calling a "
           "ShmEventLoop function";
  }
  if (__builtin_expect(!!check_tid_, false)) {
    CHECK_EQ(syscall(SYS_gettid), *check_tid_)
        << ": Being called from the wrong thread";
  }
}

// This is a bit tricky because watchers can generate new events at any time (as
// long as it's in the past). We want to check the watchers at least once before
// declaring there are no events to handle, and we want to check them again if
// event processing takes long enough that we find an event after that point in
// time to handle.
void ShmEventLoop::HandleEvent() {
  // Time through which we've checked for new events in watchers.
  monotonic_clock::time_point checked_until = monotonic_clock::min_time;
  if (!signalfd_) {
    // Nothing to check, so we can bail out immediately once we're out of
    // events.
    CHECK(watchers_.empty());
    checked_until = monotonic_clock::max_time;
  }

  // Loop until we run out of events to check.
  while (true) {
    // Time of the next event we know about. If this is before checked_until, we
    // know there aren't any new events before the next one that we already know
    // about, so no need to check the watchers.
    monotonic_clock::time_point next_time = monotonic_clock::max_time;

    if (EventCount() == 0) {
      if (checked_until != monotonic_clock::min_time) {
        // No events, and we've already checked the watchers at least once, so
        // we're all done.
        //
        // There's a small chance that a watcher has gotten another event in
        // between checked_until and now. If so, then the signalfd will be
        // triggered now and we'll re-enter HandleEvent immediately. This is
        // unlikely though, so we don't want to spend time checking all the
        // watchers unnecessarily.
        break;
      }
    } else {
      next_time = PeekEvent()->event_time();
    }
    monotonic_clock::time_point now;
    bool new_data = false;

    if (next_time > checked_until) {
      // Read all of the signals, because there's no point in waking up again
      // immediately to handle each one if we've fallen behind.
      //
      // This is safe before checking for new data on the watchers. If a signal
      // is cleared here, the corresponding CheckForNewData() call below will
      // pick it up.
      while (true) {
        const signalfd_siginfo result = signalfd_->Read();
        if (result.ssi_signo == 0) {
          break;
        }
        CHECK_EQ(result.ssi_signo, ipc_lib::kWakeupSignal);
      }
      // This is the last time we can guarantee that if a message is published
      // before, we will notice it.
      now = monotonic_clock::now();

      // Check all the watchers for new events.
      for (std::unique_ptr<WatcherState> &base_watcher : watchers_) {
        ShmWatcherState *const watcher =
            reinterpret_cast<ShmWatcherState *>(base_watcher.get());

        // Track if we got a message.
        if (watcher->CheckForNewData()) {
          new_data = true;
        }
      }
      if (EventCount() == 0) {
        // Still no events, all done now.
        break;
      }

      checked_until = now;
      // Check for any new events we found.
      next_time = PeekEvent()->event_time();
    } else {
      now = monotonic_clock::now();
    }

    if (next_time > now) {
      // Ok, we got a message with a timestamp *after* we wrote down time.  We
      // need to process it (otherwise we will go to sleep without processing
      // it), but we also need to make sure no other messages have come in
      // before it that we would process out of order.  Just go around again to
      // redo the checks.
      if (new_data) {
        continue;
      }
      break;
    }

    EventLoopEvent *const event = PopEvent();
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

    event_loops_.erase(
        std::find(event_loops_.begin(), event_loops_.end(), event_loop));

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
  CheckCurrentThread();
  SignalHandler::global()->Register(this);

  if (watchers_.size() > 0) {
    signalfd_.reset(new ipc_lib::SignalFd({ipc_lib::kWakeupSignal}));

    epoll_.OnReadable(signalfd_->fd(), [this]() { HandleEvent(); });
  }

  MaybeScheduleTimingReports();

  ReserveEvents();

  {
    logging::ScopedLogRestorer prev_logger;
    AosLogToFbs aos_logger;
    if (!skip_logger_) {
      aos_logger.Initialize(&name_, MakeSender<logging::LogMessageFbs>("/aos"));
      prev_logger.Swap(aos_logger.implementation());
    }

    aos::SetCurrentThreadName(name_.substr(0, 16));
    const cpu_set_t default_affinity = DefaultAffinity();
    if (!CPU_EQUAL(&affinity_, &default_affinity)) {
      ::aos::SetCurrentThreadAffinity(affinity_);
    }
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
    SetTimerContext(monotonic_clock::now());
    for (const auto &run : on_run_) {
      run();
    }

    // And start our main event loop which runs all the timers and handles Quit.
    epoll_.Run();

    // Once epoll exits, there is no useful nonrt work left to do.
    set_is_running(false);

    // Nothing time or synchronization critical needs to happen after this
    // point. Drop RT priority.
    ::aos::UnsetCurrentThreadRealtimePriority();
  }

  for (::std::unique_ptr<WatcherState> &base_watcher : watchers_) {
    ShmWatcherState *watcher =
        reinterpret_cast<ShmWatcherState *>(base_watcher.get());
    watcher->UnregisterWakeup();
  }

  if (watchers_.size() > 0) {
    epoll_.DeleteFd(signalfd_->fd());
    signalfd_.reset();
  }

  SignalHandler::global()->Unregister(this);

  // Trigger any remaining senders or fetchers to be cleared before destroying
  // the event loop so the book keeping matches.  Do this in the thread that
  // created the timing reporter.
  timing_report_sender_.reset();
  ClearContext();
}

void ShmEventLoop::Exit() { epoll_.Quit(); }

std::unique_ptr<ExitHandle> ShmEventLoop::MakeExitHandle() {
  return std::make_unique<ShmExitHandle>(this);
}

ShmEventLoop::~ShmEventLoop() {
  CheckCurrentThread();
  // Force everything with a registered fd with epoll to be destroyed now.
  timers_.clear();
  phased_loops_.clear();
  watchers_.clear();

  CHECK(!is_running()) << ": ShmEventLoop destroyed while running";
  CHECK_EQ(0, exit_handle_count_)
      << ": All ExitHandles must be destroyed before the ShmEventLoop";
}

void ShmEventLoop::SetRuntimeRealtimePriority(int priority) {
  CheckCurrentThread();
  if (is_running()) {
    LOG(FATAL) << "Cannot set realtime priority while running.";
  }
  priority_ = priority;
}

void ShmEventLoop::SetRuntimeAffinity(const cpu_set_t &cpuset) {
  CheckCurrentThread();
  if (is_running()) {
    LOG(FATAL) << "Cannot set affinity while running.";
  }
  affinity_ = cpuset;
}

void ShmEventLoop::set_name(const std::string_view name) {
  CheckCurrentThread();
  name_ = std::string(name);
  UpdateTimingReport();
}

absl::Span<const char> ShmEventLoop::GetWatcherSharedMemory(
    const Channel *channel) {
  CheckCurrentThread();
  ShmWatcherState *const watcher_state =
      static_cast<ShmWatcherState *>(GetWatcherState(channel));
  return watcher_state->GetSharedMemory();
}

int ShmEventLoop::NumberBuffers(const Channel *channel) {
  CheckCurrentThread();
  return ipc_lib::MakeQueueConfiguration(configuration(), channel)
      .num_messages();
}

absl::Span<char> ShmEventLoop::GetShmSenderSharedMemory(
    const aos::RawSender *sender) const {
  CheckCurrentThread();
  return static_cast<const ShmSender *>(sender)->GetSharedMemory();
}

absl::Span<const char> ShmEventLoop::GetShmFetcherPrivateMemory(
    const aos::RawFetcher *fetcher) const {
  CheckCurrentThread();
  return static_cast<const ShmFetcher *>(fetcher)->GetPrivateMemory();
}

pid_t ShmEventLoop::GetTid() {
  CheckCurrentThread();
  return syscall(SYS_gettid);
}

}  // namespace aos
