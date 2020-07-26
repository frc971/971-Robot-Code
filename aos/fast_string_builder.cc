#include "fast_string_builder.h"

namespace aos {

FastStringBuilder::FastStringBuilder(std::size_t initial_size) {
  str_.reserve(initial_size);
}

void FastStringBuilder::Reset() { str_.resize(0); }

std::string_view FastStringBuilder::Result() const {
  return std::string_view(str_);
}

std::string FastStringBuilder::MoveResult() {
  std::string result;
  result.swap(str_);
  return result;
}

void FastStringBuilder::Resize(std::size_t chars_needed) {
  str_.resize(str_.size() + chars_needed);
}

void FastStringBuilder::Append(std::string_view str) {
  std::size_t index = str_.size();
  Resize(str.size());
  str_.replace(index, str.size(), str, 0);
}

void FastStringBuilder::Append(float val) {
  std::size_t index = str_.size();
  Resize(17);
  int result = absl::SNPrintF(str_.data() + index, 17, "%.6g", val);
  PCHECK(result != -1);
  CHECK(result < 17);
  str_.resize(index + result);
}

void FastStringBuilder::Append(double val) {
  std::size_t index = str_.size();
  Resize(25);
  int result = absl::SNPrintF(str_.data() + index, 25, "%.15g", val);
  PCHECK(result != -1);
  CHECK(result < 25);
  str_.resize(index + result);
}

void FastStringBuilder::AppendChar(char val) {
  Resize(1);
  str_[str_.size() - 1] = val;
}

void FastStringBuilder::AppendBool(bool val) {
  if (bool_to_str_) {
    Append(val ? kTrue : kFalse);
  } else {
    AppendChar(val ? '1' : '0');
  }
}

std::ostream &operator<<(std::ostream &os, const FastStringBuilder &builder) {
  os.write(builder.str_.data(), builder.str_.size());
  return os;
}

}  // namespace aos
