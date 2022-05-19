// A very minimal rust_library linked into a cc_binary to make sure we still get
// the allocator symbols linked in.

#include <stdint.h>
#include <stdio.h>

extern "C" int32_t rust_return5();

extern "C" uint8_t c_return5() { return 5; }

extern "C" void c_take5(uint8_t *) {}

int main() {
  const int rust_result = rust_return5();
  printf("Rust says: %d\n", rust_result);
  if (rust_result != 5) {
    return 1;
  }
}
