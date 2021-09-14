#ifndef AOS_IPC_LIB_SHM_OBSERVERS_H_
#define AOS_IPC_LIB_SHM_OBSERVERS_H_

#include <type_traits>

namespace aos {
namespace linux_code {
namespace ipc_lib {

typedef void (*ShmAccessorObserver)(void *address, bool write);

extern ShmAccessorObserver before_observer, after_observer;

// Sets functions to run before and after SHM write operations which may
// involved multiple instructions. This is important when doing robustness
// testing because the memory has to be made writable for the whole operation,
// otherwise it never succeeds.
void SetShmAccessorObservers(ShmAccessorObserver before,
                             ShmAccessorObserver after);

// RAII class which runs before_observer during construction and after_observer
// during destruction.
class RunShmObservers {
 public:
  template <class T>
  RunShmObservers(T *address, bool write)
      : address_(static_cast<void *>(
            const_cast<typename ::std::remove_cv<T>::type *>(address))),
        write_(write) {
    if (__builtin_expect(before_observer != nullptr, false)) {
      before_observer(address_, write_);
    }
  }
  ~RunShmObservers() {
    if (__builtin_expect(after_observer != nullptr, false)) {
      after_observer(address_, write_);
    }
  }

  RunShmObservers(const RunShmObservers &) = delete;
  RunShmObservers &operator=(const RunShmObservers &) = delete;

 private:
  void *const address_;
  const bool write_;
};

}  // namespace ipc_lib
}  // namespace linux_code
}  // namespace aos

#endif  // AOS_IPC_LIB_SHM_OBSERVERS_H_
