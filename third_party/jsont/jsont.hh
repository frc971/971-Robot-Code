// JSON Tokenizer and builder. Copyright (c) 2012, Rasmus Andersson. All rights
// reserved. Use of this source code is governed by a MIT-style license that can
// be found in the LICENSE file.
#ifndef JSONT_CXX_INCLUDED
#define JSONT_CXX_INCLUDED

#include <stdint.h>  // uint8_t, int64_t
#include <stdlib.h>  // size_t
#include <string.h>  // strlen
#include <stdbool.h> // bool
#include <math.h>
#include <assert.h>
#include <string>
#include <stdexcept>

// Can haz rvalue references with move semantics?
#if (defined(_MSC_VER) && _MSC_VER >= 1600) || \
    (defined(__GXX_EXPERIMENTAL_CXX0X__) && __GXX_EXPERIMENTAL_CXX0X__) || \
    (defined(__has_feature) && __has_feature(cxx_rvalue_references))
  #define JSONT_CXX_RVALUE_REFS 1
#else
  #define JSONT_CXX_RVALUE_REFS 0
#endif

namespace jsont {

// Tokens
typedef enum {
  End = 0,       // Input ended
  ObjectStart,   // {
  ObjectEnd,     // }
  ArrayStart,    // [
  ArrayEnd,      // ]
  True,          // true
  False,         // false
  Null,          // null
  Integer,       // number value without a fraction part
  Float,         // number value with a fraction part
  String,        // string value
  FieldName,     // field name
  Error,         // An error occured (see `error()` for details)
  _Comma,
} Token;

// String encoding
typedef enum {
  UTF8TextEncoding = 0,
} TextEncoding;

// Name of `token`
const char* token_name(jsont::Token token);

class TokenizerInternal;

// Reads a sequence of bytes and produces tokens and values while doing so
class Tokenizer {
public:
  Tokenizer(const char* bytes, size_t length, TextEncoding encoding);
  ~Tokenizer();

  // Read next token
  const Token& next();

  // Access current token
  const Token& current() const;

  // Reset the tokenizer, making it possible to reuse this parser so to avoid
  // unnecessary memory allocation and deallocation.
  void reset(const char* bytes, size_t length, TextEncoding encoding);

  // True if the current token has a value
  bool hasValue() const;

  // Returns a slice of the input which represents the current value, or nothing
  // (returns 0) if the current token has no value (e.g. start of an object).
  size_t dataValue(const char const** bytes) const;

  // Returns a *copy* of the current string value.
  std::string stringValue() const;

  // Returns the current value as a double-precision floating-point number.
  double floatValue() const;

  // Returns the current value as a signed 64-bit integer.
  int64_t intValue() const;

  // Returns the current value as a boolean
  bool boolValue() const;

  // Error codes
  typedef enum {
    UnspecifiedError = 0,
    UnexpectedComma,
    UnexpectedTrailingComma,
    InvalidByte,
    PrematureEndOfInput,
    MalformedUnicodeEscapeSequence,
    MalformedNumberLiteral,
    UnterminatedString,
    SyntaxError,
  } ErrorCode;

  // Returns the error code of the last error
  ErrorCode error() const;

  // Returns a human-readable message for the last error. Never returns NULL.
  const char* errorMessage() const;

  // The byte offset into input where the tokenizer is currently looking. In the
  // event of an error, this will point to the source of the error.
  size_t inputOffset() const;

  // Total number of input bytes
  size_t inputSize() const;

  // A pointer to the input data as passed to `reset` or the constructor.
  const char* inputBytes() const;

  friend class TokenizerInternal;
private:
  size_t availableInput() const;
  size_t endOfInput() const;
  const Token& setToken(Token t);
  const Token& setError(ErrorCode error);

  struct {
    const uint8_t* bytes;
    size_t length;
    size_t offset;
  } _input;
  struct Value {
    Value() : offset(0), length(0), buffered(false) {}
    void beginAtOffset(size_t z);
    size_t offset; // into _input.bytes
    size_t length;
    std::string buffer;
    bool buffered; // if true, contents lives in buffer
  } _value;
  Token _token;
  struct {
    ErrorCode code;
  } _error;
};


// Helps in building JSON, providing a final sequential byte buffer
class Builder {
public:
  Builder() : _buf(0), _capacity(0), _size(0), _state(NeutralState) {}
  ~Builder() { if (_buf) { free(_buf); _buf = 0; } }
  Builder(const Builder& other);
  Builder& operator=(const Builder& other);
#if JSONT_CXX_RVALUE_REFS
  Builder(Builder&& other);
  Builder& operator=(Builder&& other);
#endif

  Builder& startObject();
  Builder& endObject();
  Builder& startArray();
  Builder& endArray();
  Builder& fieldName(const char* v, size_t length, TextEncoding e=UTF8TextEncoding);
  Builder& fieldName(const std::string& name, TextEncoding enc=UTF8TextEncoding);
  Builder& value(const char* v, size_t length, TextEncoding e=UTF8TextEncoding);
  Builder& value(const char* v);
  Builder& value(const std::string& v);
  Builder& value(double v);
  Builder& value(int64_t v);
  Builder& value(int v);
  Builder& value(unsigned int v);
  Builder& value(long v);
  Builder& value(bool v);
  Builder& nullValue();

  size_t size() const;
  const char* bytes() const;
  std::string toString() const;
  const char* seizeBytes(size_t& size_out);
  const void reset();

private:
  size_t available() const;
  void reserve(size_t size);
  void prefix();
  Builder& appendString(const uint8_t* v, size_t length, TextEncoding enc);
  Builder& appendChar(char byte);

  char*  _buf;
  size_t _capacity;
  size_t _size;
  enum {
    NeutralState = 0,
    AfterFieldName,
    AfterValue,
    AfterObjectStart,
    AfterArrayStart,
  } _state;
};


// Convenience function
inline Builder build() { return Builder(); }


// ------------------- internal ---------------------

inline Tokenizer::Tokenizer(const char* bytes, size_t length,
    TextEncoding encoding) : _token(End) {
  reset(bytes, length, encoding);
}

inline const Token& Tokenizer::current() const { return _token; }

inline bool Tokenizer::hasValue() const {
  return _token >= Integer && _token <= FieldName;
}

inline std::string Tokenizer::stringValue() const {
  const char* bytes;
  size_t size = dataValue(&bytes);
  return std::string(bytes, size);
}

inline bool Tokenizer::boolValue() const {
  return _token == True;
}

inline size_t Tokenizer::availableInput() const {
  return _input.length - _input.offset;
}
inline size_t Tokenizer::endOfInput() const {
  return _input.offset == _input.length;
}
inline const Token& Tokenizer::setToken(Token t) {
  return _token = t;
}
inline const Token& Tokenizer::setError(Tokenizer::ErrorCode error) {
  _error.code = error;
  return _token = Error;
}
inline size_t Tokenizer::inputOffset() const {
  return _input.offset;
}
inline size_t Tokenizer::inputSize() const {
  return _input.length;
}
inline const char* Tokenizer::inputBytes() const {
  return (const char*)_input.bytes;
}

inline void Tokenizer::Value::beginAtOffset(size_t z) {
  offset = z;
  length = 0;
  buffered = false;
}

inline Tokenizer::ErrorCode Tokenizer::error() const {
  return _error.code;
}


inline Builder& Builder::startObject() {
  prefix();
  _state = AfterObjectStart;
  return appendChar('{');
}

inline Builder& Builder::endObject() {
  _state = AfterValue;
  return appendChar('}');
}

inline Builder& Builder::startArray() {
  prefix();
  _state = AfterArrayStart;
  return appendChar('[');
}

inline Builder& Builder::endArray() {
  _state = AfterValue;
  return appendChar(']');
}

inline Builder& Builder::fieldName(const std::string& name, TextEncoding enc) {
  return fieldName(name.data(), name.size(), enc);
}

inline Builder& Builder::fieldName(const char* v, size_t length,
    TextEncoding enc) {
  prefix();
  _state = AfterFieldName;
  return appendString((const uint8_t*)v, length, enc);
}

inline Builder& Builder::value(const char* v, size_t length, TextEncoding enc) {
  prefix();
  _state = AfterValue;
  return appendString((const uint8_t*)v, length, enc);
}

inline Builder& Builder::value(const char* v) {
  return value(v, strlen(v));
}

inline Builder& Builder::value(const std::string& v) {
  return value(v.data(), v.size());
}

inline Builder& Builder::value(double v) {
  prefix();
  reserve(256);
  int z = snprintf(_buf+_size, 256, "%g", v);
  assert(z < 256);
  _size += z;
  _state = AfterValue;
  return *this;
}

inline Builder& Builder::value(int64_t v) {
  prefix();
  reserve(21);
  int z = snprintf(_buf+_size, 21, "%lld", v);
  assert(z < 21);
  _size += z;
  _state = AfterValue;
  return *this;
}

inline Builder& Builder::value(int v) { return value((int64_t)v); }
inline Builder& Builder::value(unsigned int v) { return value((int64_t)v); }
inline Builder& Builder::value(long v) { return value((int64_t)v); }

inline Builder& Builder::value(bool v) {
  prefix();
  if (v) {
    reserve(4);
    _buf[_size]   = 't';
    _buf[++_size] = 'r';
    _buf[++_size] = 'u';
    _buf[++_size] = 'e';
    ++_size;
  } else {
    reserve(5);
    _buf[_size]   = 'f';
    _buf[++_size] = 'a';
    _buf[++_size] = 'l';
    _buf[++_size] = 's';
    _buf[++_size] = 'e';
    ++_size;
  }
  _state = AfterValue;
  return *this;
}

inline Builder& Builder::nullValue() {
  prefix();
  reserve(4);
  _buf[_size]   = 'n';
  _buf[++_size] = 'u';
  _buf[++_size] = 'l';
  _buf[++_size] = 'l';
  ++_size;
  _state = AfterValue;
  return *this;
}

inline size_t Builder::size() const { return _size; }
inline const char* Builder::bytes() const { return _buf; }
inline std::string Builder::toString() const {
  return std::string(bytes(), size());
}
inline const char* Builder::seizeBytes(size_t& size_out) {
  const char* buf = _buf;
  size_out = _size;
  _buf = 0;
  _capacity = 0;
  reset();
  return buf;
}
inline const void Builder::reset() {
  _size = 0;
  _state = NeutralState;
}

inline size_t Builder::available() const {
  return _capacity - _size;
}

inline void Builder::reserve(size_t size) {
  if (available() < size) {
    #if 0
    // exact allocation for debugging purposes
    printf("DEBUG Builder::reserve: size=%zu available=%zu grow_by=%zu\n",
      size, available(), (size - available()) );
    _capacity += size - available();
    #else
    _capacity += size - available();
    _capacity = (_capacity < 64) ? 64 : (_capacity * 1.5);
    #endif
    _buf = (char*)realloc((void*)_buf, _capacity);
  }
}

inline void Builder::prefix() {
  if (_state == AfterFieldName) {
    appendChar(':');
  } else if (_state == AfterValue) {
    appendChar(',');
  }
}

inline Builder& Builder::appendChar(char byte) {
  reserve(1);
  _buf[_size++] = byte;
  return *this;
}

}

#endif // JSONT_CXX_INCLUDED
