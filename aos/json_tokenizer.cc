#include "aos/json_tokenizer.h"

#include <cerrno>
#include <limits>

namespace aos {

void Tokenizer::ConsumeWhitespace() {
  while (true) {
    if (AtEnd()) {
      return;
    }
    // Skip any whitespace.
    if (Char() == ' ' || Char() == '\r' || Char() == '\t') {
      ConsumeChar();
    } else if (Char() == '\n') {
      ConsumeChar();
      ++linenumber_;
    } else if (Consume("/*")) {
      while (!Consume("*/")) {
        if (Char() == '\n') {
          ++linenumber_;
        }
        ConsumeChar();
      }
    } else if (Consume("//")) {
      // C++ style comment.  Keep consuming chars until newline, or until the
      // end of the file if this is the last line (no newline at end of file).
      while (true) {
        ConsumeChar();
        if (AtEnd()) {
          return;
        }
        if (Char() == '\n') {
          ++linenumber_;
          break;
        }
      }
    } else {
      // There is no fail.  Once we are out of whitespace (including 0 of it),
      // declare success.
      return;
    }
  }
}

bool Tokenizer::Consume(const char *token) {
  const std::string_view original = data_;
  while (true) {
    // Finishing the token is success.
    if (*token == '\0') {
      return true;
    }

    // But finishing the data first is failure.
    if (AtEnd()) {
      data_ = original;
      return false;
    }

    // Missmatch is failure.
    if (*token != Char()) {
      data_ = original;
      return false;
    }

    ConsumeChar();
    ++token;
  }
}

bool Tokenizer::ConsumeString(::std::string *s) {
  // Under no conditions is it acceptible to run out of data while parsing a
  // string.  Any AtEnd checks should confirm that.
  const std::string_view original = data_;
  if (AtEnd()) {
    return false;
  }

  // Expect the leading "
  if (Char() != '"') {
    return false;
  }

  ConsumeChar();
  std::string_view last_parsed_data = data_;
  *s = ::std::string();

  while (true) {
    if (AtEnd()) {
      data_ = original;
      return false;
    }

    // If we get an end or an escape, do something special.
    if (Char() == '"' || Char() == '\\') {
      // Save what we found up until now, not including this character.
      *s += ::std::string(
          last_parsed_data.substr(0, last_parsed_data.size() - data_.size()));

      // Update the pointer.
      last_parsed_data = data_;

      // " is the end, declare victory.
      if (Char() == '"') {
        ConsumeChar();
        if (unicode_high_surrogate_ != -1) {
          fprintf(stderr, "Invalid unicode - Unpaired high surrogate\n");
          data_ = original;
          return false;
        }
        return true;
      } else {
        ConsumeChar();
        // Now consume valid escape characters and add their representation onto
        // the output string.
        if (AtEnd()) {
          data_ = original;
          return false;
        } else if (Char() == '"') {
          *s += "\"";
        } else if (Char() == '\\') {
          *s += "\\";
        } else if (Char() == '/') {
          *s += "/";
        } else if (Char() == 'b') {
          *s += "\b";
        } else if (Char() == 'f') {
          *s += "\f";
        } else if (Char() == 'n') {
          *s += "\n";
        } else if (Char() == 'r') {
          *s += "\r";
        } else if (Char() == 't') {
          *s += "\t";
        } else if (Char() == 'u') {
          if (!ConsumeUnicode(s)) {
            fprintf(stderr, "Invalid unicode on line %d\n", linenumber_);
            data_ = original;
            return false;
          }
        }
      }
      // And skip the escaped character.
      last_parsed_data = data_.substr(1);
    }

    ConsumeChar();
  }
}

bool Tokenizer::ConsumeUnicode(::std::string *s) {
  // Under no conditions is it acceptible to run out of data while parsing a
  // unicode.  Any AtEnd checks should confirm that.
  uint32_t val;

  // Consume unicode representation
  ConsumeChar();

  char target[5];

  // Valid unicode is 4 hex digits so evaluate the next 4 characters
  for (int count = 0; count < 4; count++) {
    // If there is no data or data is an invalid char, return false
    if (AtEnd()) {
      return false;
    }

    if (!isxdigit(Char())) {
      return false;
    }

    target[count] = Char();

    // Do not consume the last character
    if (count == 3) {
      break;
    }

    ConsumeChar();
  }
  target[4] = '\0';

  // References: flatbuffers/src/idl_parser.cpp
  val = flatbuffers::StringToUInt(target, 16);

  if (val >= 0xD800 && val <= 0xDBFF) {
    if (unicode_high_surrogate_ != -1) {
      fprintf(stderr, "Invalid unicode - Multiple high surrogates\n");
      return false;
    } else {
      unicode_high_surrogate_ = static_cast<int>(val);
    }
  } else if (val >= 0xDC00 && val <= 0xDFFF) {
    if (unicode_high_surrogate_ == -1) {
      fprintf(stderr, "Invalid unicode - Unpaired low surrogate\n");
      return false;
    } else {
      int code_point =
          0x10000 + ((unicode_high_surrogate_ & 0x03FF) << 10) + (val & 0x03FF);
      flatbuffers::ToUTF8(code_point, s);
      unicode_high_surrogate_ = -1;
    }
  } else {
    if (unicode_high_surrogate_ != -1) {
      fprintf(stderr, "Invalid unicode - Unpaired high surrogate\n");
      return false;
    }
    flatbuffers::ToUTF8(static_cast<int>(val), s);
  }
  return true;
}

bool Tokenizer::ConsumeNumber(::std::string *s) {
  // Under no conditions is it acceptible to run out of data while parsing a
  // number.  Any AtEnd() checks should confirm that.
  *s = ::std::string();
  const std::string_view original = data_;

  // Consume the leading - unconditionally.
  Consume("-");

  // See if we find nan.  This isn't standards compliant, but is what
  // flatbuffers prints out, so we need to parse it.
  if (Consume("nan")) {
    *s = ::std::string(original.substr(0, original.size() - data_.size()));
    return true;
  }

  // People tend to use null instead of nan.  Accept that too.
  if (Consume("null")) {
    *s = ::std::string("nan");
    return true;
  }

  // Inf is also acceptable.
  if (Consume("inf")) {
    *s = ::std::string(original.substr(0, original.size() - data_.size()));
    return true;
  }

  // Then, we either get a 0, or we get a nonzero.  Only nonzero can be followed
  // by a second number.
  if (!Consume("0")) {
    if (AtEnd()) {
      return false;
    } else if (Char() >= '1' && Char() <= '9') {
      // This wasn't a zero, but was a valid digit.  Consume it.
      ConsumeChar();
    } else {
      return false;
    }

    // Now consume any number of any digits.
    while (true) {
      if (AtEnd()) {
        data_ = original;
        return false;
      }
      if (Char() < '0' || Char() > '9') {
        break;
      }
      ConsumeChar();
    }
  }

  // We could now have a decimal.
  if (Char() == '.') {
    ConsumeChar();
    while (true) {
      if (AtEnd()) {
        data_ = original;
        return false;
      }
      // And any number of digits.
      if (Char() < '0' || Char() > '9') {
        break;
      }
      ConsumeChar();
    }
  }

  // And now an exponent.
  if (Char() == 'e' || Char() == 'E') {
    ConsumeChar();
    if (AtEnd()) {
      data_ = original;
      return false;
    }

    // Which could have a +-
    if (Char() == '+' || Char() == '-') {
      ConsumeChar();
    }
    int count = 0;
    while (true) {
      if (AtEnd()) {
        data_ = original;
        return false;
      }
      // And digits.
      if (Char() < '0' || Char() > '9') {
        break;
      }
      ConsumeChar();
      ++count;
    }
    // But, it is an error to have an exponent and nothing following it.
    if (count == 0) {
      data_ = original;
      return false;
    }
  }

  *s = ::std::string(original.substr(0, original.size() - data_.size()));
  return true;
}

Tokenizer::TokenType Tokenizer::Next() {
  switch (state_) {
    case State::kExpectObjectStart:
      // We should always start out with a {
      if (!Consume("{")) {
        fprintf(stderr, "Error on line %d, expected { for start.\n",
                linenumber_);
        return TokenType::kError;
      }

      // Document that we just started an object.
      object_type_.push_back(ObjectType::kObject);

      ConsumeWhitespace();

      if (Consume("}")) {
        ConsumeWhitespace();
        state_ = State::kExpectObjectEnd;
      } else {
        state_ = State::kExpectField;
      }
      return TokenType::kStartObject;

    case State::kExpectField: {
      // Fields are built up of strings, whitespace, and then a : (followed by
      // whitespace...)
      ::std::string s;
      if (!ConsumeString(&s)) {
        fprintf(stderr, "Error on line %d, expected string for field name.\n",
                linenumber_);
        if (Consume("}")) {
          fprintf(stderr,
                  "Got '}' instead.  Did you add an extra trailing ','?\n");
        }
        return TokenType::kError;
      }
      field_name_ = ::std::move(s);

      ConsumeWhitespace();

      if (!Consume(":")) {
        fprintf(stderr, "Error on line %d, expected ':', got '%c'\n",
                linenumber_, Char());
        return TokenType::kError;
      }

      ConsumeWhitespace();

      state_ = State::kExpectValue;

      return TokenType::kField;
    } break;
    case State::kExpectValue: {
      TokenType result = TokenType::kError;

      ::std::string s;
      if (Consume("{")) {
        // Fields are in objects.  Record and recurse.
        object_type_.push_back(ObjectType::kObject);

        ConsumeWhitespace();

        // And then if we encounter the end again, go to the end state.
        if (Consume("}")) {
          ConsumeWhitespace();
          state_ = State::kExpectObjectEnd;
        } else {
          state_ = State::kExpectField;
        }
        return TokenType::kStartObject;
      } else if (Consume("[")) {
        // Values are in arrays.  Record and recurse.
        object_type_.push_back(ObjectType::kArray);

        ConsumeWhitespace();
        state_ = State::kExpectValue;
        return TokenType::kStartArray;
      } else if (ConsumeString(&s)) {
        // Parsed as a string, grab it.
        field_value_ = ::std::move(s);
        result = TokenType::kStringValue;
      } else if (ConsumeNumber(&s)) {
        // Parsed as a number, grab it.
        field_value_ = ::std::move(s);
        result = TokenType::kNumberValue;
      } else if (Consume("true")) {
        // Parsed as a true, grab it.
        field_value_ = "true";
        result = TokenType::kTrueValue;
      } else if (Consume("false")) {
        // Parsed as a false, grab it.
        field_value_ = "false";
        result = TokenType::kFalseValue;
      } else {
        switch (object_type_.back()) {
          case ObjectType::kObject:
            if (Consume("}")) {
              ConsumeWhitespace();
              state_ = State::kExpectObjectEnd;
              return Next();
            }
            break;
          case ObjectType::kArray:
            if (Consume("]")) {
              ConsumeWhitespace();
              state_ = State::kExpectArrayEnd;
              return Next();
            }
            break;
        }
        // Couldn't parse, so we have a syntax error.
        fprintf(stderr, "Error line %d, invalid field value.\n", linenumber_);
      }

      ConsumeWhitespace();

      // After a field, we either have a , and another field (or value if we are
      // in an array), or we should be closing out the object (or array).
      if (Consume(",")) {
        ConsumeWhitespace();
        switch (object_type_.back()) {
          case ObjectType::kObject:
            state_ = State::kExpectField;
            break;
          case ObjectType::kArray:
            state_ = State::kExpectValue;
            break;
        }
      } else {
        // Sanity check that the stack is deep enough.
        if (object_type_.size() == 0) {
          fprintf(stderr, "Error on line %d\n", linenumber_);
          return TokenType::kError;
        }

        // And then require closing out the object.
        switch (object_type_.back()) {
          case ObjectType::kObject:
            if (Consume("}")) {
              ConsumeWhitespace();
              state_ = State::kExpectObjectEnd;
            } else {
              fprintf(stderr, "Error on line %d, expected } or ,\n",
                      linenumber_);
              return TokenType::kError;
            }
            break;
          case ObjectType::kArray:
            if (Consume("]")) {
              ConsumeWhitespace();
              state_ = State::kExpectArrayEnd;
            } else {
              fprintf(stderr, "Error on line %d, expected ] or ,\n",
                      linenumber_);
              return TokenType::kError;
            }
            break;
        }
      }
      return result;
    } break;

    case State::kExpectArrayEnd:
    case State::kExpectObjectEnd: {
      const TokenType result = state_ == State::kExpectArrayEnd
                                   ? TokenType::kEndArray
                                   : TokenType::kEndObject;
      // This is a transient state so we can send 2 tokens out in a row.  We
      // discover the object or array end at the end of reading the value.
      object_type_.pop_back();
      if (object_type_.size() == 0) {
        // We unwound the outer object.  We should send kEnd next.
        state_ = State::kExpectEnd;
      } else if (object_type_.back() == ObjectType::kObject) {
        // If we are going into an object, it should either have another field
        // or end.
        if (Consume(",")) {
          ConsumeWhitespace();
          state_ = State::kExpectField;
        } else if (Consume("}")) {
          ConsumeWhitespace();
          state_ = State::kExpectObjectEnd;
        } else {
          fprintf(stderr, "Error on line %d, expected , or }\n", linenumber_);
          return TokenType::kError;
        }
      } else if (object_type_.back() == ObjectType::kArray) {
        // If we are going into an array, it should either have another value
        // or end.
        if (Consume(",")) {
          ConsumeWhitespace();
          state_ = State::kExpectValue;
        } else if (Consume("]")) {
          ConsumeWhitespace();
          state_ = State::kExpectArrayEnd;
        } else {
          fprintf(stderr, "Error on line %d, expected , or ]\n", linenumber_);
          return TokenType::kError;
        }
      }
      // And then send out the correct token.
      return result;
    }
    case State::kExpectEnd:
      // If we are supposed to be done, confirm nothing is after the end.
      if (AtEnd()) {
        return TokenType::kEnd;
      } else {
        fprintf(stderr, "Data past end at line %d\n", linenumber_);
        return TokenType::kError;
      }
  }
  return TokenType::kError;
}

bool Tokenizer::FieldAsInt(long long *value) {
  const char *pos = field_value().c_str();
  errno = 0;
  *value = strtoll(field_value().c_str(), const_cast<char **>(&pos), 10);
  if (pos != field_value().c_str() + field_value().size() || errno != 0) {
    return false;
  }
  return true;
}

bool Tokenizer::FieldAsDouble(double *value) {
  const char *pos = field_value().c_str();
  errno = 0;
  if (field_value() == "nan") {
    *value = std::numeric_limits<double>::quiet_NaN();
    return true;
  } else if (field_value() == "-nan") {
    *value = -std::numeric_limits<double>::quiet_NaN();
    return true;
  }

  if (field_value() == "inf") {
    *value = std::numeric_limits<double>::infinity();
    return true;
  } else if (field_value() == "-inf") {
    *value = -std::numeric_limits<double>::infinity();
    return true;
  }

  *value = strtod(field_value().c_str(), const_cast<char **>(&pos));

  if (pos != field_value().c_str() + field_value().size() || errno != 0) {
    return false;
  }
  return true;
}

}  // namespace aos
