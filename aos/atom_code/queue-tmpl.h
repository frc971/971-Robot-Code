namespace aos {

template <class T>
bool ScopedMessagePtr<T>::Send() {
  assert(msg_ != NULL);
  msg_->SetTimeToNow();
  assert(queue_ != NULL);
  bool return_value = aos_queue_write_msg_free(queue_, msg_, OVERRIDE) == 0;
  msg_ = NULL;
  return return_value;
}

template <class T>
bool ScopedMessagePtr<T>::SendBlocking() {
  assert(msg_ != NULL);
  msg_->SetTimeToNow();
  assert(queue_ != NULL);
  bool return_value = aos_queue_write_msg_free(queue_, msg_, BLOCK) == 0;
  msg_ = NULL;
  return return_value;
}

template <class T>
void ScopedMessagePtr<T>::reset(T *msg) {
  if (queue_ != NULL && msg_ != NULL) {
    aos_queue_free_msg(queue_, msg_);
  }
  msg_ = msg;
}

// A SafeScopedMessagePtr<> manages a message pointer.
// It frees it properly when the ScopedMessagePtr<> goes out of scope or gets
// sent.  By design, there is no way to get the ScopedMessagePtr to release the
// message pointer.  When the message gets sent, it allocates a queue message,
// copies the data into it, and then sends it.  Copies copy the message.
template <class T>
class SafeScopedMessagePtr {
 public:
  // Returns a pointer to the message.
  // This stays valid until Send or the destructor have been called.
  T *get() { return msg_; }

  T &operator*() {
    T *msg = get();
    assert(msg != NULL);
    return *msg;
  }

  T *operator->() {
    T *msg = get();
    assert(msg != NULL);
    return msg;
  }

#ifndef SWIG
  operator bool() {
    return msg_ != NULL;
  }

  const T *get() const { return msg_; }

  const T &operator*() const {
    const T *msg = get();
    assert(msg != NULL);
    return *msg;
  }

  const T *operator->() const {
    const T *msg = get();
    assert(msg != NULL);
    return msg;
  }
#endif  // SWIG

  // Sends the message and removes our reference to it.
  // If the queue is full, over-rides the oldest message in it with our new
  // message.
  // Returns true on success, and false otherwise.
  // The message will be freed.
  bool Send() {
    assert(msg_ != NULL);
    assert(queue_ != NULL);
    msg_->SetTimeToNow();
    T *shm_msg = static_cast<T *>(aos_queue_get_msg(queue_));
    if (shm_msg == NULL) {
      return false;
    }
    *shm_msg = *msg_;
    bool return_value =
        aos_queue_write_msg_free(queue_, shm_msg, OVERRIDE) == 0;
    reset();
    return return_value;
  }

  // Sends the message and removes our reference to it.
  // If the queue is full, blocks until it is no longer full.
  // Returns true on success, and false otherwise.
  // Frees the message.
  bool SendBlocking() {
    assert(msg_ != NULL);
    assert(queue_ != NULL);
    msg_->SetTimeToNow();
    T *shm_msg = static_cast<T *>(aos_queue_get_msg(queue_));
    if (shm_msg == NULL) {
      return false;
    }
    *shm_msg = *msg_;
    bool return_value = aos_queue_write_msg_free(queue_, shm_msg, BLOCK) == 0;
    reset();
    return return_value;
  }

  // Frees the contained message.
  ~SafeScopedMessagePtr() {
    reset();
  }

#ifndef SWIG
  // Implements a move constructor to take the message pointer from the
  // temporary object to save work.
  SafeScopedMessagePtr(SafeScopedMessagePtr<T> &&ptr)
    : queue_(ptr.queue_),
      msg_(ptr.msg_) {
    ptr.msg_ = NULL;
  }
#endif  // SWIG

  // Copy constructor actually copies the data.
  SafeScopedMessagePtr(const SafeScopedMessagePtr<T> &ptr)
      : queue_(ptr.queue_),
        msg_(NULL) {
    reset(new T(*ptr.get()));
  }
#ifndef SWIG
  // Equal operator copies the data.
  void operator=(const SafeScopedMessagePtr<T> &ptr) {
    queue_ = ptr.queue_;
    reset(new T(*ptr.get()));
  }
#endif  // SWIG

 private:
  // Provide access to private constructor.
  friend class aos::Queue<typename std::remove_const<T>::type>;
  friend class aos::SafeMessageBuilder<T>;

  // Only Queue should be able to build a message pointer.
  SafeScopedMessagePtr(aos_queue *queue)
      : queue_(queue), msg_(new T()) {}

  // Sets the pointer to msg, freeing the old value if it was there.
  // This is private because nobody should be able to get a pointer to a message
  // that needs to be scoped without using the queue.
  void reset(T *msg = NULL) {
    if (msg_) {
      delete msg_;
    }
    msg_ = msg;
  }

  // Sets the queue that owns this message.
  void set_queue(aos_queue *queue) { queue_ = queue; }

  // The queue that the message is a part of.
  aos_queue *queue_;
  // The message or NULL.
  T *msg_;
};

template <class T>
void Queue<T>::Init() {
  if (queue_ == NULL) {
    // Signature of the message.
    aos_type_sig kQueueSignature{sizeof(T), T::kHash, T::kQueueLength};

    queue_ = aos_fetch_queue(queue_name_, &kQueueSignature);
    queue_msg_.set_queue(queue_);
  }
}

template <class T>
void Queue<T>::Clear() {
  if (queue_ == NULL) {
    queue_msg_.reset();
    queue_ = NULL;
    queue_msg_.set_queue(NULL);
  }
}

template <class T>
bool Queue<T>::FetchNext() {
  Init();
  // TODO(aschuh): Use aos_queue_read_msg_index so that multiple readers
  // reading don't randomly get only part of the messages.
  // Document here the tradoffs that are part of each method.
  const T *msg = static_cast<const T *>(aos_queue_read_msg(queue_,
        NON_BLOCK));
  // Only update the internal pointer if we got a new message.
  if (msg != NULL) {
    queue_msg_.reset(msg);
  }
  return msg != NULL;
}

template <class T>
bool Queue<T>::FetchNextBlocking() {
  Init();
  const T *msg = static_cast<const T *>(aos_queue_read_msg(queue_, BLOCK));
  queue_msg_.reset(msg);
  assert (msg != NULL);
  return true;
}

template <class T>
bool Queue<T>::FetchLatest() {
  Init();
  const T *msg = static_cast<const T *>(aos_queue_read_msg(queue_,
        FROM_END | NON_BLOCK | PEEK));
  // Only update the internal pointer if we got a new message.
  if (msg != NULL && msg != queue_msg_.get()) {
    queue_msg_.reset(msg);
    return true;
  }
  // The message has to get freed if we didn't use it (and aos_queue_free_msg is
  // ok to call on NULL).
  aos_queue_free_msg(queue_, msg);
  return false;
}

template <class T>
SafeScopedMessagePtr<T> Queue<T>::SafeMakeMessage() {
  Init();
  SafeScopedMessagePtr<T> safe_msg(queue_);
  safe_msg->Zero();
  return safe_msg;
}

template <class T>
ScopedMessagePtr<T> Queue<T>::MakeMessage() {
  Init();
  return ScopedMessagePtr<T>(queue_, MakeRawMessage());
}

template <class T>
T *Queue<T>::MakeRawMessage() {
  T *ret = static_cast<T *>(aos_queue_get_msg(queue_));
  assert(ret != NULL);
  return ret;
}

template <class T>
aos::MessageBuilder<T> Queue<T>::MakeWithBuilder() {
  Init();
  return aos::MessageBuilder<T>(queue_, MakeRawMessage());
}


// This builder uses the safe message pointer so that it can be safely copied
// and used by SWIG or in places where it could be leaked.
template <class T>
class SafeMessageBuilder {
 public:
  typedef T Message;
  bool Send();
};

template <class T>
aos::SafeMessageBuilder<T> Queue<T>::SafeMakeWithBuilder() {
  Init();
  return aos::SafeMessageBuilder<T>(queue_);
}


}  // namespace aos
