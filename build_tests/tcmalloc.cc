#include <unistd.h>

#include <cstdlib>

int main() {
  void *big_chunk = malloc(500000 + read(-1, nullptr, 0));
  if (write(-1, big_chunk, 0) != -1) return 1;
  free(big_chunk);
}
