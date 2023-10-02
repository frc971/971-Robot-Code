#ifndef AOS_INIT_FOR_RUST_H_
#define AOS_INIT_FOR_RUST_H_

#include <string>
#include <string_view>
#include <vector>

#include "aos/for_rust.h"
#include "cxx.h"

namespace aos {

struct FlagInfo {
  std::string name_;
  std::string type_;
  std::string description_;
  std::string default_value_;
  std::string filename_;

  rust::Str name() const {
    return StringViewToRustStr(std::string_view(name_));
  }
  rust::Str ty() const { return StringViewToRustStr(std::string_view(type_)); }
  rust::Str description() const {
    return StringViewToRustStr(std::string_view(description_));
  }
  rust::Str default_value() const {
    return StringViewToRustStr(std::string_view(default_value_));
  }
  rust::Str filename() const {
    return StringViewToRustStr(std::string_view(filename_));
  }
};

// A special initialization function that initializes the C++ parts in a way
// compatible with Rust. This requires careful coordination with `:init_rs`, do
// not use it from anywhere else.
void InitFromRust(const char *argv0);

std::vector<FlagInfo> GetCppFlags();

bool SetCommandLineOption(const char *name, const char *value);
std::string GetCommandLineOption(const char *name);

}  // namespace aos

#endif  // AOS_INIT_FOR_RUST_H_
