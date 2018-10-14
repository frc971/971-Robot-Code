#ifndef AOS_ONCE_H_
#define AOS_ONCE_H_

#include <stdint.h>

#include "aos/gtest_prod.h"

namespace aos {
namespace testing {

FORWARD_DECLARE_TEST_CASE(OnceTest, MemoryClearing);

}  // namespace testing

// Designed for the same thing as pthread_once: to run something exactly 1 time.
//
// Intended use case:
//   const char *CalculateSomethingCool() {
//     static ::aos::Once<const char> once(DoCalculateSomethingCool);
//     return once.Get();
//   }
//
// IMPORTANT: Instances _must_ be placed in memory that gets 0-initialized
// automatically or Reset() must be called exactly once!!
// The expected use case is to use one of these as a static variable, and those
// do get 0-initialized under the C++ standard. Global variables are treated the
// same way by the C++ Standard.
// Placing an instance in shared memory (and using Reset()) is also supported.
// The constructor does not initialize all of the member variables!
// This is because initializing them in the constructor creates a race condition
// if initialization of static variables isn't thread safe.
template<typename T>
class Once {
 public:
  typedef T *(*Function)();
  explicit Once(Function function);

  // Returns the result of calling function_. The first call will actually run
  // it and then any other ones will block (if necessary) until it's finished
  // and then return the same thing.
  T *Get();

  // Will clear out all the member variables. If this is going to be used
  // instead of creating an instance in 0-initialized memory, then this method
  // must be called exactly once before Get() is called anywhere.
  // This method can also be called to run the function again the next time
  // Get() is called. However, calling it again is not thread safe.
  void Reset();

 private:
  // The function to run to calculate result_.
  Function function_;
  // Whether or not it is running. Gets atomically swapped from false to true by
  // the thread that actually runs function_.
  bool run_;
  // Whether or not it is done. Gets set to true after the thread that is
  // running function_ finishes running it and storing the result in result_.
  bool done_;
  // What function_ returned when it was executed.
  T *result_;

  FRIEND_TEST_NAMESPACE(OnceTest, MemoryClearing, testing);
};

}  // namespace aos

#include "aos/once-tmpl.h"

#endif  // AOS_ONCE_H_
