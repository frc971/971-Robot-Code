#ifndef AOS_JSON_TO_FLATBUFFER_H_
#define AOS_JSON_TO_FLATBUFFER_H_

#include <cstddef>
#include <string>

#include "flatbuffers/flatbuffers.h"

namespace aos {

// Parses the flatbuffer into the vector, or returns an empty vector.
::std::vector<uint8_t> JsonToFlatbuffer(
    const char *data, const flatbuffers::TypeTable *typetable);

// Converts a flatbuffer into a Json string.
//
// multi_line controls if the Json is written out on multiple lines or one.
::std::string FlatbufferToJson(const uint8_t *buffer,
                               const flatbuffers::TypeTable *typetable,
                               bool multi_line = false);

// This class implements the state machine at json.org
class Tokenizer {
 public:
  Tokenizer(const char *data) : data_(data) {}

  enum class TokenType {
    kEnd,
    kError,
    kStartObject,
    kEndObject,
    kStartArray,
    kEndArray,
    kField,
    kNumberValue,
    kStringValue,
    kTrueValue,
    kFalseValue,
  };

  // Returns the next token.
  TokenType Next();

  // Returns the last field_name and field_value.  These are only valid when
  // Next returns them.
  const ::std::string &field_name() const { return field_name_; }
  const ::std::string &field_value() const { return field_value_; }

  // Parses the current field value as a long long.  Returns false if it failed
  // to parse.
  bool FieldAsInt(long long *value);
  // Parses the current field value as a double.  Returns false if it failed
  // to parse.
  bool FieldAsDouble(double *value);

  // Returns true if we are at the end of the input.
  bool AtEnd() { return *data_ == '\0'; }

  const char *data_left() const { return data_; }

 private:
  // Consumes a string out of data_.  Populates s with the string.  Returns true
  // if a valid string was found, and false otherwise.
  // data_ is updated only on success.
  bool ConsumeString(::std::string *s);
  // Consumes a number out of data_.  Populates s with the string containing the
  // number.  Returns true if a valid number was found, and false otherwise.
  // data_ is updated only on success.
  bool ConsumeNumber(::std::string *s);
  // Consumes a fixed token out of data_. Returns true if the string was found,
  // and false otherwise.
  // data_ is updated only on success.
  bool Consume(const char* token);
  // Consumes whitespace out of data_. Returns true if the string was found,
  // and false otherwise.
  // data_ is unconditionally updated.
  void ConsumeWhitespace();

  // State for the parsing state machine.
  enum class State {
    kExpectField,
    kExpectObjectStart,
    kExpectObjectEnd,
    kExpectArrayEnd,
    kExpectValue,
    kExpectEnd,
  };

  State state_ = State::kExpectObjectStart;

  // Data pointer.
  const char *data_;
  // Current line number used for printing debug.
  int linenumber_ = 0;

  // Stack used to track which object type we were in when we recursed.
  enum class ObjectType {
    kObject,
    kArray,
  };
  ::std::vector<ObjectType> object_type_;

  // Last field name.
  ::std::string field_name_;
  // Last field value.
  ::std::string field_value_;
};
}  // namespace aos

#endif  // AOS_JSON_TO_FLATBUFFER_H_
