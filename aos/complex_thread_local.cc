#include "aos/complex_thread_local.h"

#include <pthread.h>

#include "aos/die.h"
#include "absl/base/call_once.h"

#define SIMPLE_CHECK(call)              \
  do {                                  \
    const int value = call;             \
    if (value != 0) {                   \
      PRDie(value, "%s failed", #call); \
    }                                   \
  } while (false)

namespace aos {
namespace {

void ExecuteDestructorList(void *v) {
  for (const ComplexThreadLocalDestructor *c =
           static_cast<ComplexThreadLocalDestructor *>(v);
       c != nullptr; c = c->next) {
    c->function(c->param);
  }
}

void CreateKey(pthread_key_t **r) {
  static pthread_key_t hr;
  SIMPLE_CHECK(pthread_key_create(&hr, ExecuteDestructorList));
  *r = &hr;
}

absl::once_flag key_once;

pthread_key_t *GetKey() {
  static pthread_key_t *key = nullptr;
  absl::call_once(key_once, CreateKey, &key);
  return key;
}
} // namespace

void ComplexThreadLocalDestructor::Add() {
  static_assert(
      ::std::is_pod<ComplexThreadLocalDestructor>::value,
      "ComplexThreadLocalDestructor might not be safe to pass through void*");
  pthread_key_t *key = GetKey();

  next = static_cast<ComplexThreadLocalDestructor *>(pthread_getspecific(*key));
  SIMPLE_CHECK(pthread_setspecific(*key, this));
}

void ComplexThreadLocalDestructor::Remove() {
  pthread_key_t *key = GetKey();

  ComplexThreadLocalDestructor *previous = nullptr;
  for (ComplexThreadLocalDestructor *c =
           static_cast<ComplexThreadLocalDestructor *>(
               pthread_getspecific(*key));
       c != nullptr; c = c->next) {
    if (c == this) {
      // If it's the first one.
      if (previous == nullptr) {
        SIMPLE_CHECK(pthread_setspecific(*key, next));
      } else {
        previous->next = next;
      }
      return;
    }
    previous = c;
  }
  ::aos::Die("%p is not in the destructor list\n", this);
}

}  // namespace aos
