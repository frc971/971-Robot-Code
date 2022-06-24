namespace aos::util {
template <typename T, typename SharedState>
ThreadedQueue<T, SharedState>::ThreadedQueue(
    std::function<PushResult(SharedState)> push_request_handler,
    SharedState initial_state)
    : popped_(&mutex_),
      pushed_(&mutex_),
      state_(initial_state),
      pusher_thread_([this, push_request_handler]() {
        while (true) {
          PushResult result = push_request_handler(State());
          {
            MutexLocker locker(&mutex_);
            done_ = done_ || result.done;
            if (result.item.has_value()) {
              queue_.push(std::move(result.item.value()));
            }
            pushed_.Broadcast();
            if (done_) {
              return;
            }
            if (result.more_to_push || state_updated_) {
              continue;
            } else {
              pusher_waiting_ = true;
              CHECK(!popped_.Wait());
              pusher_waiting_ = false;
            }
          }
        }
      }) {}

template <typename T, typename SharedState>
ThreadedQueue<T, SharedState>::~ThreadedQueue() {
  StopPushing();
  pusher_thread_.join();
}

template <typename T, typename SharedState>
void ThreadedQueue<T, SharedState>::WaitForNoMoreWork() {
  MutexLocker locker(&mutex_);
  while (state_updated_ || (!pusher_waiting_ && !done_)) {
    CHECK(!pushed_.Wait());
  }
}

template <typename T, typename SharedState>
SharedState ThreadedQueue<T, SharedState>::State() {
  MutexLocker locker(&mutex_);
  state_updated_ = false;
  return state_;
}

template <typename T, typename SharedState>
void ThreadedQueue<T, SharedState>::SetState(const SharedState &state) {
  MutexLocker locker(&mutex_);
  state_ = state;
  state_updated_ = true;
  popped_.Broadcast();
}

template <typename T, typename SharedState>
void ThreadedQueue<T, SharedState>::StopPushing() {
  // Ensure that the mutex is locked before doing anything, to make sure that
  // the pushing thread actually observes the change.
  MutexLocker locker(&mutex_);
  done_ = true;
  popped_.Broadcast();
}

template <typename T, typename SharedState>
std::optional<T> ThreadedQueue<T, SharedState>::Peek() {
  return PeekOrPop(false);
}

template <typename T, typename SharedState>
std::optional<T> ThreadedQueue<T, SharedState>::Pop() {
  return PeekOrPop(true);
}

template <typename T, typename SharedState>
std::optional<T> ThreadedQueue<T, SharedState>::PeekOrPop(bool pop) {
  MutexLocker locker(&mutex_);
  while (!done_ && queue_.empty()) {
    CHECK(!pushed_.Wait());
  }
  if (queue_.empty()) {
    return std::nullopt;
  }
  if (pop) {
    T result = std::move(queue_.front());
    queue_.pop();
    popped_.Broadcast();
    return result;
  } else {
    return queue_.front();
  }
}
}  // namespace aos::util
