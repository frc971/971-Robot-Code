#ifndef AOS_LINUX_CODE_COMPLEX_THREAD_LOCAL_H_
#define AOS_LINUX_CODE_COMPLEX_THREAD_LOCAL_H_

#include <assert.h>

#include <type_traits>
#include <utility>

namespace aos {

// Instances form a (per-thread) list of destructor functions to call when the
// thread exits.
// Only ComplexThreadLocal should use this.
struct ComplexThreadLocalDestructor {
  // Adds this to the list of destructors in this thread.
  void Add();
  // Removes this from the list of destructors in this thread. ::aos::Dies if it
  // is not there.
  void Remove();

  void (*function)(void *);
  void *param;

  ComplexThreadLocalDestructor *next;
};

// Handles creating a thread-local (per type) object with non-trivial
// constructor and/or destructor. It will be correctly destroyed on thread exit.
//
// Each thread using an instantiation of this class has its own independent slot
// for storing a T. An instance of T is not actually constructed until a thread
// calls Create, after which a pointer to it will be returned from get() etc
// until after Clear is called.
//
// Example usage:
// class Something {
//  private:
//   class Data {
//    public:
//     Data(const ::std::string &value) : value_(value) {}
//
//     int DoSomething() {
//       if (cached_result_ == 0) {
//         // Do something expensive with value_ and store it in
//         // cached_result_.
//       }
//       return cached_result_;
//     }
//
//    private:
//     const ::std::string value_;
//     int cached_result_ = 0;
//   };
//   ComplexThreadLocal<Data> thread_local_;
//   ::std::string a_string_;
//
//   int DoSomething() {
//     thread_local_.Create(a_string_);
//     return thread_local_->DoSomething();
//   }
// };
//
// The current implementation is based on
// <http://stackoverflow.com/questions/12049684/gcc-4-7-on-linux-pthreads-nontrivial-thread-local-workaround-using-thread-n>.
// TODO(brians): Change this to just simple standard C++ thread_local once all
// of our compilers have support.
template <typename T>
class ComplexThreadLocal {
 public:
  // Actually creates the object in this thread if there is not one there
  // already.
  // args are all perfectly forwarded to the constructor.
  template <typename... Args>
  void Create(Args &&... args) {
    if (initialized) return;
    new (&storage) T(::std::forward<Args>(args)...);
    destructor.function = PlacementDelete;
    destructor.param = &storage;
    destructor.Add();
    initialized = true;
  }

  // Removes the object in this thread (if any), including calling its
  // destructor.
  void Clear() {
    if (!initialized) return;
    destructor.Remove();
    PlacementDelete(&storage);
    initialized = false;
  }

  // Returns true if there is already an object in this thread.
  bool created() const { return initialized; }

  // Returns the object currently created in this thread or nullptr.
  T *operator->() const {
    return get();
  }
  T *get() const {
    if (initialized) {
      return static_cast<T *>(static_cast<void *>(&storage));
    } else {
      return nullptr;
    }
  }

 private:
  typedef typename ::std::aligned_storage<
      sizeof(T), ::std::alignment_of<T>::value>::type Storage;

  // Convenient helper for calling a destructor.
  static void PlacementDelete(void *t) { static_cast<T *>(t)->~T(); }

  // True iff this storage has been initialized.
  static __thread bool initialized;
  // Where we actually store the object for this thread (if any).
  static __thread Storage storage;
  // The linked list element representing this storage.
  static __thread ComplexThreadLocalDestructor destructor;
};

template <typename T>
__thread bool ComplexThreadLocal<T>::initialized;
template <typename T>
__thread typename ComplexThreadLocal<T>::Storage ComplexThreadLocal<T>::storage;
template <typename T>
__thread ComplexThreadLocalDestructor ComplexThreadLocal<T>::destructor;

}  // namespace aos

#endif  // AOS_LINUX_CODE_COMPLEX_THREAD_LOCAL_H_
