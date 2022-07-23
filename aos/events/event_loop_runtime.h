#ifndef AOS_EVENTS_EVENT_LOOP_RUNTIME_H_
#define AOS_EVENTS_EVENT_LOOP_RUNTIME_H_

// Exposes the primitives to implement an async Rust runtime on top of an
// EventLoop. This is not intended to be used directly, so the APIs are not
// particularly ergonomic for C++. See the Rust wrapper for detailed
// documentation.

#include <memory>
#include <optional>

#include "aos/events/event_loop.h"
#include "aos/for_rust.h"
#include "cxx.h"

namespace aos {

// An alternative version of Context to feed autocxx, to work around
// https://github.com/google/autocxx/issues/787.
/// <div rustbindgen replaces="aos::Context"></div>
struct RustContext {
  int64_t monotonic_event_time;
  int64_t realtime_event_time;

  int64_t monotonic_remote_time;
  int64_t realtime_remote_time;

  uint32_t queue_index;
  uint32_t remote_queue_index;

  size_t size;
  const void *data;

  int buffer_index;

  // Work around https://github.com/google/autocxx/issues/266.
  uint8_t source_boot_uuid[16];
};

static_assert(sizeof(Context) == sizeof(RustContext));
static_assert(alignof(Context) == alignof(RustContext));
static_assert(offsetof(Context, monotonic_event_time) ==
              offsetof(RustContext, monotonic_event_time));
static_assert(offsetof(Context, realtime_event_time) ==
              offsetof(RustContext, realtime_event_time));
static_assert(offsetof(Context, monotonic_remote_time) ==
              offsetof(RustContext, monotonic_remote_time));
static_assert(offsetof(Context, realtime_remote_time) ==
              offsetof(RustContext, realtime_remote_time));
static_assert(offsetof(Context, queue_index) ==
              offsetof(RustContext, queue_index));
static_assert(offsetof(Context, remote_queue_index) ==
              offsetof(RustContext, remote_queue_index));
static_assert(offsetof(Context, size) == offsetof(RustContext, size));
static_assert(offsetof(Context, data) == offsetof(RustContext, data));
static_assert(offsetof(Context, buffer_index) ==
              offsetof(RustContext, buffer_index));
static_assert(offsetof(Context, source_boot_uuid) ==
              offsetof(RustContext, source_boot_uuid));
static_assert(sizeof(Context::source_boot_uuid) ==
              sizeof(RustContext::source_boot_uuid));
static_assert(sizeof(RustContext) == sizeof(Context),
              "Update this when adding or removing fields");

// Similar to Rust's `Future<Output = Never>`.
class ApplicationFuture {
 public:
  ApplicationFuture() = default;
  virtual ~ApplicationFuture() = default;

  // Calls a Rust `Future::poll`, with a waker that will panic if used. Because
  // our Future's Output is Never, the inner Rust implementation can only return
  // Poll::Pending, which is equivalent to void.
  virtual void Poll() = 0;
};

// Similar to Rust's `Stream<Item = const Option&>`.
class WatcherForRust {
 public:
  WatcherForRust(std::unique_ptr<RawFetcher> fetcher)
      : fetcher_(std::move(fetcher)) {}
  ~WatcherForRust() = default;

  const Context *PollNext() {
    if (!fetcher_->FetchNext()) {
      return nullptr;
    }
    return &fetcher_->context();
  }

 private:
  const std::unique_ptr<RawFetcher> fetcher_;
};

class SenderForRust {
 public:
  SenderForRust(std::unique_ptr<RawSender> sender)
      : sender_(std::move(sender)) {}
  ~SenderForRust() = default;

  uint8_t *data() { return reinterpret_cast<uint8_t *>(sender_->data()); }
  size_t size() { return sender_->size(); }
  RawSender::Error SendBuffer(size_t size) { return sender_->Send(size); }
  RawSender::Error CopyAndSend(const uint8_t *data, size_t size) {
    return sender_->Send(data, size);
  }

 private:
  const std::unique_ptr<RawSender> sender_;
};

class FetcherForRust {
 public:
  FetcherForRust(std::unique_ptr<RawFetcher> fetcher)
      : fetcher_(std::move(fetcher)) {}
  ~FetcherForRust() = default;

  bool FetchNext() { return fetcher_->FetchNext(); }
  bool Fetch() { return fetcher_->Fetch(); }

  const Context &context() const { return fetcher_->context(); }

 private:
  const std::unique_ptr<RawFetcher> fetcher_;
};

class EventLoopRuntime {
 public:
  EventLoopRuntime(EventLoop *event_loop) : event_loop_(event_loop) {}
  ~EventLoopRuntime() = default;

  EventLoop *event_loop() { return event_loop_; }

  void spawn(std::unique_ptr<ApplicationFuture> task) {
    CHECK(!task_) << ": May only call spawn once";
    task_ = std::move(task);
    // TODO(Brian): Do this once we've got OnRun support.
    // DoPoll();
    // TODO(Brian): Once we have OnRun support, should this move there or stay
    // here unconditionally?
    event_loop_->OnRun([this] { DoPoll(); });
  }

  const Configuration *configuration() const {
    return event_loop_->configuration();
  }
  const Node *node() const { return event_loop_->node(); }

  // autocxx generates broken C++ code for `time_point`, see
  // https://github.com/google/autocxx/issues/787.
  int64_t monotonic_now() const {
    return std::chrono::nanoseconds(
               event_loop_->monotonic_now().time_since_epoch())
        .count();
  }
  int64_t realtime_now() const {
    return std::chrono::nanoseconds(
               event_loop_->realtime_now().time_since_epoch())
        .count();
  }

  rust::Str name() const { return StringViewToRustStr(event_loop_->name()); }

  WatcherForRust MakeWatcher(const Channel *channel) {
    event_loop_->MakeRawNoArgWatcher(channel,
                                     [this](const Context &) { DoPoll(); });
    return WatcherForRust(event_loop_->MakeRawFetcher(channel));
  }

  SenderForRust MakeSender(const Channel *channel) {
    return SenderForRust(event_loop_->MakeRawSender(channel));
  }

  FetcherForRust MakeFetcher(const Channel *channel) {
    return FetcherForRust(event_loop_->MakeRawFetcher(channel));
  }

 private:
  // Polls the top-level future once. This is what all the callbacks should do.
  void DoPoll() {
    if (task_) {
      task_->Poll();
    }
  }

  EventLoop *const event_loop_;

  std::unique_ptr<ApplicationFuture> task_;
};

}  // namespace aos

#endif  // AOS_EVENTS_EVENT_LOOP_RUNTIME_H_
