namespace aos {

template <class T>
bool ScopedMessagePtr<T>::Send() {
  assert(msg_ != NULL);
  msg_->SetTimeToNow();
  assert(queue_ != NULL);
  bool return_value = queue_->WriteMessage(msg_, RawQueue::kOverride);
  msg_ = NULL;
  return return_value;
}

template <class T>
bool ScopedMessagePtr<T>::SendBlocking() {
  assert(msg_ != NULL);
  msg_->SetTimeToNow();
  assert(queue_ != NULL);
  bool return_value = queue_->WriteMessage(msg_, RawQueue::kBlock);
  msg_ = NULL;
  return return_value;
}

template <class T>
void ScopedMessagePtr<T>::reset(T *msg) {
  if (queue_ != NULL && msg_ != NULL) {
    queue_->FreeMessage(msg_);
  }
  msg_ = msg;
}

template <class T>
void Queue<T>::Init() {
  if (queue_ == NULL) {
    queue_ = RawQueue::Fetch(queue_name_, sizeof(T),
                             static_cast<int>(T::kHash),
                             T::kQueueLength);
    queue_msg_.set_queue(queue_);
  }
}

template <class T>
void Queue<T>::Clear() {
  if (queue_ != NULL) {
    queue_msg_.reset();
    queue_ = NULL;
    queue_msg_.set_queue(NULL);
  }
  index_ = 0;
}

template <class T>
bool Queue<T>::FetchNext() {
  Init();
  const T *msg = static_cast<const T *>(
      queue_->ReadMessageIndex(RawQueue::kNonBlock, &index_));
  // Only update the internal pointer if we got a new message.
  if (msg != NULL) {
    queue_msg_.reset(msg);
  }
  return msg != NULL;
}

template <class T>
void Queue<T>::FetchNextBlocking() {
  Init();
  const T *msg = static_cast<const T *>(
      queue_->ReadMessageIndex(RawQueue::kBlock, &index_));
  queue_msg_.reset(msg);
  assert (msg != NULL);
}

template <class T>
bool Queue<T>::FetchLatest() {
  Init();
  static constexpr Options<RawQueue> kOptions =
      RawQueue::kFromEnd | RawQueue::kNonBlock;
  const T *msg =
      static_cast<const T *>(queue_->ReadMessageIndex(kOptions, &index_));
  // Only update the internal pointer if we got a new message.
  if (msg != NULL && msg != queue_msg_.get()) {
    queue_msg_.reset(msg);
    return true;
  }
  // The message has to get freed if we didn't use it (and RawQueue::FreeMessage
  // is ok to call on NULL).
  queue_->FreeMessage(msg);
  return false;
}

template <class T>
void Queue<T>::FetchAnother() {
  if (!FetchLatest()) FetchNextBlocking();
}

template <class T>
ScopedMessagePtr<T> Queue<T>::MakeMessage() {
  Init();
  return ScopedMessagePtr<T>(queue_, MakeRawMessage());
}

template <class T>
T *Queue<T>::MakeRawMessage() {
  T *ret = static_cast<T *>(queue_->GetMessage());
  assert(ret != NULL);
  ret->Zero();
  return ret;
}

template <class T>
aos::MessageBuilder<T> Queue<T>::MakeWithBuilder() {
  Init();
  T *const ret = MakeRawMessage();
  return aos::MessageBuilder<T>(queue_, ret);
}

}  // namespace aos
