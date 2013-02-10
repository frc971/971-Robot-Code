namespace aos {

// The easiest way to hack this together is to have the scoped msg pointer not
// manage the pointer, since it is a pointer to the only msg in the queue.
template <class T>
bool ScopedMessagePtr<T>::Send() {
  msg_->SetTimeToNow();
  reset();
  return true;
}

template <class T>
bool ScopedMessagePtr<T>::SendBlocking() {
  msg_->SetTimeToNow();
  reset();
  return true;
}

template <class T>
void ScopedMessagePtr<T>::reset(T *msg) {
  msg_ = msg;
}

template <class T>
void Queue<T>::Init() {}

template <class T>
bool Queue<T>::FetchNext() {
  Init();
  return true;
}

template <class T>
bool Queue<T>::FetchNextBlocking() {
  Init();
  return true;
}

template <class T>
bool Queue<T>::FetchLatest() {
  Init();
  return true;
}

template <class T>
ScopedMessagePtr<T> Queue<T>::MakeMessage() {
  Init();
  return ScopedMessagePtr<T>(&msg_);
}

template <class T>
aos::MessageBuilder<T> Queue<T>::MakeWithBuilder() {
  Init();
  return aos::MessageBuilder<T>(&msg_);
}

}  // namespace aos
