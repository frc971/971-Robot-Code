#ifndef AOS_EVENTS_GLIB_MAIN_LOOP_H_
#define AOS_EVENTS_GLIB_MAIN_LOOP_H_

#include <glib-object.h>
#include <glib.h>

#include <unordered_set>

#include "aos/events/shm_event_loop.h"

namespace aos {

class GlibMainLoop;

// Adapts a std::function to a g_source-style callback.
//
// T is the function pointer type.
//
// This doesn't interact with a GlibMainLoop, so it's safe to use from any
// thread, but it also won't catch lifetime bugs cleanly.
template <typename T>
class GlibSourceCallback {
 private:
  template <typename TResult, typename... TArgs>
  struct helper {
    static TResult Invoke(TArgs... args, gpointer user_data) {
      GlibSourceCallback *const pointer =
          reinterpret_cast<GlibSourceCallback *>(user_data);
      CHECK(g_main_context_is_owner(pointer->g_main_context_))
          << ": Callback being called from the wrong thread";
      return pointer->function_(args...);
    }
    using Function = std::function<TResult(TArgs...)>;
  };
  // A helper to deduce template arguments (type template arguments can't be
  // deduced, so we need a function).
  template <typename TResult>
  static helper<TResult> MakeHelper(TResult (*)(gpointer)) {
    return helper<TResult>();
  }

  using HelperType =
      decltype(GlibSourceCallback::MakeHelper(std::declval<T>()));

 protected:
  using Function = typename HelperType::Function;

 public:
  // May be called from any thread.
  GlibSourceCallback(Function function, GSource *source,
                     GMainContext *g_main_context);
  // May only be called from the main thread.
  ~GlibSourceCallback();

  // Instances may not be moved because a pointer to the instance gets passed
  // around.
  GlibSourceCallback(const GlibSourceCallback &) = delete;
  GlibSourceCallback &operator=(const GlibSourceCallback &) = delete;

 private:
  GSourceFunc g_source_func() const {
    return reinterpret_cast<GSourceFunc>(&HelperType::Invoke);
  }
  gpointer user_data() const {
    return const_cast<gpointer>(reinterpret_cast<const void *>(this));
  }

  const Function function_;
  GSource *const source_;
  GMainContext *const g_main_context_;
};

template <typename T>
class GlibSourceCallbackRefcount : public GlibSourceCallback<T> {
 public:
  GlibSourceCallbackRefcount(typename GlibSourceCallback<T>::Function function,
                             GSource *source, GlibMainLoop *glib_main_loop);
  ~GlibSourceCallbackRefcount();

 private:
  GlibMainLoop *const glib_main_loop_;
};

// Adapts a std::function to a g_signal-style callback. This includes calling
// the std::function the main thread, vs the g_signal callback is invoked on an
// arbitrary thread.
template <typename... Args>
class GlibSignalCallback {
 public:
  GlibSignalCallback(std::function<void(Args...)> function,
                     GlibMainLoop *glib_main_loop, gpointer instance,
                     const char *detailed_signal);
  ~GlibSignalCallback();

  // Instances may not be moved because a pointer to the instance gets passed
  // around.
  GlibSignalCallback(const GlibSignalCallback &) = delete;
  GlibSignalCallback &operator=(const GlibSignalCallback &) = delete;

 private:
  static void InvokeSignal(Args... args, gpointer user_data);
  gpointer user_data() const {
    return const_cast<gpointer>(reinterpret_cast<const void *>(this));
  }

  const std::function<void(Args...)> function_;
  GlibMainLoop *const glib_main_loop_;
  const gpointer instance_;
  const gulong signal_handler_id_;

  // Protects changes to invocations_ and idle_callback_.
  aos::stl_mutex lock_;
  std::vector<std::tuple<Args...>> invocations_;
  std::optional<GlibSourceCallback<GSourceFunc>> idle_callback_;
};

// Manages a GMainLoop attached to a ShmEventLoop.
//
// Also provides C++ RAII wrappers around the related glib objects.
class GlibMainLoop {
 public:
  GlibMainLoop(ShmEventLoop *event_loop);
  ~GlibMainLoop();
  GlibMainLoop(const GlibMainLoop &) = delete;
  GlibMainLoop &operator=(const GlibMainLoop &) = delete;

  GMainContext *g_main_context() { return g_main_context_; }
  GMainLoop *g_main_loop() { return g_main_loop_; }

  auto AddIdle(std::function<gboolean()> callback) {
    return GlibSourceCallbackRefcount<GSourceFunc>(std::move(callback),
                                                   g_idle_source_new(), this);
  }

  auto AddTimeout(std::function<gboolean()> callback, guint interval) {
    return GlibSourceCallbackRefcount<GSourceFunc>(
        std::move(callback), g_timeout_source_new(interval), this);
  }

  // Connects a glib signal to a callback. Note that this is NOT a Unix signal.
  //
  // Note that the underlying signal handler is called in one of gstreamer's
  // thread, but callback will be called in the main thread. This means that any
  // objects being passed in with the expectation of the handler incrementing
  // their refcount need special handling. This also means any glib signal
  // handlers which need to return a value cannot use this abstraction.
  //
  // It's recommended to pass an actual std::function (NOT something with a
  // user-defined conversion to a std::function, such as a lambda) as the first
  // argument, which allows the template arguments to be deduced.
  //
  // Passing a lambda with explicit template arguments doesn't work
  // unfortunately... I think it's because the variadic template argument could
  // be extended beyond anything you explicitly pass in, so it's always doing
  // deduction, and deduction doesn't consider the user-defined conversion
  // between the lambda's type and the relevant std::function type. C++ sucks,
  // sorry.
  template <typename... Args>
  auto ConnectSignal(std::function<void(Args...)> callback, gpointer instance,
                     const char *detailed_signal) {
    return GlibSignalCallback<Args...>(std::move(callback), this, instance,
                                       detailed_signal);
  }

  void AddChild() { ++children_; }

  void RemoveChild() {
    CHECK_GT(children_, 0);
    --children_;
  }

 private:
  void RemoveAllFds();
  void BeforeWait();

  // fds which we have added to the epoll object.
  std::unordered_set<int> added_fds_;

  ShmEventLoop *const event_loop_;
  TimerHandler *const timeout_timer_;
  GMainContext *const g_main_context_;
  GMainLoop *const g_main_loop_;

  // The list of FDs and priority received from glib on the latest
  // g_main_context_query call, so we can pass them to the g_main_context_check
  // next time around.
  std::vector<GPollFD> gpoll_fds_;
  gint last_query_max_priority_;

  // Tracks whether we did the call to acquire g_main_context_.
  bool acquired_context_ = false;

  // Tracking all the child glib objects we create. None of them should outlive
  // us, and asserting that helps catch bugs in application code that leads to
  // use-after-frees.
  int children_ = 0;
};

template <typename T>
inline GlibSourceCallback<T>::GlibSourceCallback(Function function,
                                                 GSource *source,
                                                 GMainContext *g_main_context)
    : function_(function), source_(source), g_main_context_(g_main_context) {
  g_source_set_callback(source_, g_source_func(), user_data(), nullptr);
  CHECK_GT(g_source_attach(source_, g_main_context_), 0u);
  VLOG(1) << "Attached source " << source_ << " to " << g_main_context_;
}

template <typename T>
inline GlibSourceCallback<T>::~GlibSourceCallback() {
  CHECK(g_main_context_is_owner(g_main_context_))
      << ": May only be destroyed from the main thread";

  g_source_destroy(source_);
  VLOG(1) << "Destroyed source " << source_;
  // Now, the callback won't be called any more (because this source is no
  // longer attached to a context), even if refcounts remain that hold the
  // source itself alive. That's not safe in a multithreaded context, but we
  // only allow this operation in the main thread, which means it synchronizes
  // with any other code in the main thread that might call the callback.

  g_source_unref(source_);
}

template <typename T>
GlibSourceCallbackRefcount<T>::GlibSourceCallbackRefcount(
    typename GlibSourceCallback<T>::Function function, GSource *source,
    GlibMainLoop *glib_main_loop)
    : GlibSourceCallback<T>(std::move(function), source,
                            glib_main_loop->g_main_context()),
      glib_main_loop_(glib_main_loop) {
  glib_main_loop_->AddChild();
}

template <typename T>
GlibSourceCallbackRefcount<T>::~GlibSourceCallbackRefcount() {
  glib_main_loop_->RemoveChild();
}

template <typename... Args>
GlibSignalCallback<Args...>::GlibSignalCallback(
    std::function<void(Args...)> function, GlibMainLoop *glib_main_loop,
    gpointer instance, const char *detailed_signal)
    : function_(std::move(function)),
      glib_main_loop_(glib_main_loop),
      instance_(instance),
      signal_handler_id_(g_signal_connect(
          instance, detailed_signal,
          G_CALLBACK(&GlibSignalCallback::InvokeSignal), user_data())) {
  CHECK_GT(signal_handler_id_, 0);
  VLOG(1) << this << " connected glib signal with " << user_data() << " as "
          << signal_handler_id_ << " on " << instance << ": "
          << detailed_signal;
  glib_main_loop_->AddChild();
}

template <typename... Args>
GlibSignalCallback<Args...>::~GlibSignalCallback() {
  g_signal_handler_disconnect(instance_, signal_handler_id_);
  VLOG(1) << this << " disconnected glib signal on " << instance_ << ": "
          << signal_handler_id_;
  glib_main_loop_->RemoveChild();
}

template <typename... Args>
void GlibSignalCallback<Args...>::InvokeSignal(Args... args,
                                               gpointer user_data) {
  CHECK(user_data != nullptr) << ": invalid glib signal callback";
  GlibSignalCallback *const pointer =
      reinterpret_cast<GlibSignalCallback *>(user_data);
  VLOG(1) << "Adding invocation of signal " << pointer;
  std::unique_lock<aos::stl_mutex> locker(pointer->lock_);
  CHECK_EQ(!!pointer->idle_callback_, !pointer->invocations_.empty());
  if (!pointer->idle_callback_) {
    // If we don't already have a callback set, then schedule a new one.
    pointer->idle_callback_.emplace(
        [pointer]() {
          std::unique_lock<aos::stl_mutex> locker(pointer->lock_);
          for (const auto &args : pointer->invocations_) {
            VLOG(1) << "Calling signal handler for " << pointer;
            std::apply(pointer->function_, args);
          }
          pointer->invocations_.clear();
          pointer->idle_callback_.reset();
          return false;
        },
        g_idle_source_new(), pointer->glib_main_loop_->g_main_context());
  }
  pointer->invocations_.emplace_back(
      std::make_tuple<Args...>(std::forward<Args>(args)...));
}

}  // namespace aos

#endif  // AOS_EVENTS_GLIB_MAIN_LOOP_H_
