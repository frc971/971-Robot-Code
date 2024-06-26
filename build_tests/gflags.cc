#include "absl/flags/flag.h"

ABSL_FLAG(int32_t, test_flag, 0, "Test command-line flag");

int main() { return absl::GetFlag(FLAGS_test_flag); }
