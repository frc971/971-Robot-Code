#include "jsont.hh"

namespace jsont {

static const int8_t kHexValueTable[55] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, // 0-0
  -1, -1, -1, -1, -1, -1, -1,
  10, 11, 12, 13, 14, 15, // A-F
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1,
  10, 11, 12, 13, 14, 15 // a-f
};

static uint64_t _xtou64(const uint8_t* bytes, size_t len) {
  uint64_t value = 0;
  uint64_t cutoff = UINT64_MAX / 16;
  int cutoff_digit = (int)(UINT64_MAX - cutoff * 16);

  for (size_t i = 0; i != len; ++i) {
    uint8_t b = bytes[i];
    int8_t digit = (b > '0'-1 && b < 'f'+1) ? kHexValueTable[b-'0'] : -1;
    if (b == -1 || // bad digit
        (value > cutoff) || // overflow
        ((value == cutoff) && (digit > cutoff_digit)) ) {
      return UINT64_MAX;
    } else {
      value = (value * 16) + digit;
    }
  }

  return value;
}


#ifdef NAN
  #define _JSONT_NAN NAN
#else
  #define _JSONT_NAN nan(0)
#endif


const char* token_name(jsont::Token tok) {
  switch (tok) {
    case End:         return "End";
    case ObjectStart: return "ObjectStart";
    case ObjectEnd:   return "ObjectEnd";
    case ArrayStart:  return "ArrayStart";
    case ArrayEnd:    return "ArrayEnd";
    case True:        return "True";
    case False:       return "False";
    case Null:        return "Null";
    case Integer:     return "Integer";
    case Float:       return "Float";
    case String:      return "String";
    case FieldName:   return "FieldName";
    default:                 return "?";
  }
}


class TokenizerInternal {
public:
  inline static const uint8_t* currentInput(const Tokenizer& self) {
    return self._input.bytes + self._input.offset;
  }

  inline static const Token& readAtom(Tokenizer& self, const char* str,
        size_t len, const Token& token) {
    if (self.availableInput() < len) {
      return self.setError(Tokenizer::PrematureEndOfInput);
    } else if (memcmp(currentInput(self), str, len) != 0) {
      return self.setError(Tokenizer::InvalidByte);
    } else {
      self._input.offset += len;
      return self.setToken(token);
    }
  }
};


Tokenizer::~Tokenizer() {}


void Tokenizer::reset(const char* bytes, size_t length, TextEncoding encoding) {
  assert(encoding == UTF8TextEncoding); // only supported encoding
  _input.bytes = (const uint8_t*)bytes;
  _input.length = length;
  _input.offset = 0;
  _error.code = UnspecifiedError;
  // Advance to first token
  next();
}


const char* Tokenizer::errorMessage() const {
  switch (_error.code) {
    case UnexpectedComma:
      return "Unexpected comma";
    case UnexpectedTrailingComma:
      return "Unexpected trailing comma";
    case InvalidByte:
      return "Invalid input byte";
    case PrematureEndOfInput:
      return "Premature end of input";
    case MalformedUnicodeEscapeSequence:
      return "Malformed Unicode escape sequence";
    case MalformedNumberLiteral:
      return "Malformed number literal";
    case UnterminatedString:
      return "Unterminated string";
    case SyntaxError:
      return "Illegal JSON (syntax error)";
    default:
      return "Unspecified error";
  }
}


size_t Tokenizer::dataValue(const char const** bytes) const {
  if (!hasValue()) { return 0; }
  if (_value.buffered) {
    *bytes = (const char const*)_value.buffer.data();
    return _value.buffer.size();
  } else {
    *bytes = (const char const*)(_input.bytes + _value.offset);
    return _value.length;
  }
}


double Tokenizer::floatValue() const {
  if (!hasValue()) {
    return _token == jsont::True ? 1.0 : 0.0;
  }

  const char* bytes;

  if (_value.buffered) {
    // edge-case since only happens with string values using escape sequences
    bytes = _value.buffer.c_str();
  } else {
    bytes = (const char*)_input.bytes + _value.offset;
    if (availableInput() == 0) {
      // In this case where the data lies at the edge of the buffer, we can't pass
      // it directly to atof, since there will be no sentinel byte. We are fine
      // with a copy, since this is an edge case (only happens either for broken
      // JSON or when the whole document is just a number).
      char* buf[128];
      if (_value.length > 127) {
        // We are unable to interpret such a large literal in this edge-case
        return _JSONT_NAN;
      }
      memcpy((void*)buf, (const void*)bytes, _value.length);
      buf[_value.length] = '\0';
      return strtod((const char*)buf, (char**)0);
    }
  }

  return strtod(bytes, (char**)0);
}


int64_t Tokenizer::intValue() const {
  if (!hasValue()) {
    return _token == jsont::True ? 1LL : 0LL;
  }

  const char* bytes;

  if (_value.buffered) {
    // edge-case since only happens with string values using escape sequences
    bytes = _value.buffer.c_str();
  } else {
    bytes = (const char*)_input.bytes + _value.offset;
    if (availableInput() == 0) {
      // In this case where the data lies at the edge of the buffer, we can't pass
      // it directly to atof, since there will be no sentinel byte. We are fine
      // with a copy, since this is an edge case (only happens either for broken
      // JSON or when the whole document is just a number).
      char* buf[21];
      if (_value.length > 20) {
        // We are unable to interpret such a large literal in this edge-case
        return 0;
      }
      memcpy((void*)buf, (const void*)bytes, _value.length);
      buf[_value.length] = '\0';
      return strtoll((const char*)buf, (char**)0, 10);
    }
  }

  return strtoll(bytes, (char**)0, 10);
}


const Token& Tokenizer::next() {
  //
  // { } [ ] n t f "
  //         | | | |
  //         | | | +- /[^"]*/ "
  //         | | +- a l s e
  //         | +- r u e
  //         +- u l l
  //
  while (!endOfInput()) {
    uint8_t b = _input.bytes[_input.offset++];
    switch (b) {
      case '{': return setToken(ObjectStart);
      case '}': {
        if (_token == _Comma) { return setError(UnexpectedTrailingComma); }
        return setToken(ObjectEnd);
      }

      case '[': return setToken(ArrayStart);
      case ']': {
        if (_token == _Comma) { return setError(UnexpectedTrailingComma); }
        return setToken(ArrayEnd);
      }

      case 'n':
        return TokenizerInternal::readAtom(*this, "ull", 3, jsont::Null);
      case 't':
        return TokenizerInternal::readAtom(*this, "rue", 3, jsont::True);
      case 'f':
        return TokenizerInternal::readAtom(*this, "alse", 4, jsont::False);

      case ' ': case '\t': case '\r': case '\n': // IETF RFC4627
        // ignore whitespace and let the outer "while" do its thing
        break;

      case 0:
        return setError(InvalidByte);

      // when we read a value, we don't produce a token until we either reach
      // end of input, a colon (then the value is a field name), a comma, or an
      // array or object terminator.

      case '"': {
        _value.beginAtOffset(_input.offset);

        while (!endOfInput()) {
          b = _input.bytes[_input.offset++];
          assert(_input.offset < _input.length);
          
          switch (b) {

            case '\\': {
              // We must go buffered since the input segment != value
              if (!_value.buffered) {
                _value.buffered = true;
                _value.buffer.assign(
                  (const char*)(_input.bytes+_value.offset),
                  _input.offset - _value.offset - 1
                );
              }

              if (endOfInput()) {
                return setError(PrematureEndOfInput);
              }
              
              b = _input.bytes[_input.offset++];
              switch (b) {
                case 'b': _value.buffer.append(1, '\x08'); break;
                case 'f': _value.buffer.append(1, '\x0C'); break;
                case 'n': _value.buffer.append(1, '\x0A'); break;
                case 'r': _value.buffer.append(1, '\x0D'); break;
                case 't': _value.buffer.append(1, '\x09'); break;
                case 'u': {
                  // \uxxxx
                  if (availableInput() < 4) {
                    return setError(PrematureEndOfInput);
                  }

                  uint64_t utf16cp =
                    _xtou64(TokenizerInternal::currentInput(*this), 4);
                  _input.offset += 4;

                  if (utf16cp > 0xffff) {
                    return setError(MalformedUnicodeEscapeSequence);
                  }

                  uint16_t cp = (uint16_t)(0xffff & utf16cp);

                  // Append UTF-8 byte(s) representing the Unicode codepoint cp
                  if (cp < 0x80) {
                    // U+0000 - U+007F
                    uint8_t cp8 = ((uint8_t)cp);
                    _value.buffer.append(1, (char)cp8);
                  } else if (cp < 0x800) {
                    // U+0080 - U+07FF
                    uint8_t cp8 = (uint8_t)((cp >> 6) | 0xc0);
                    _value.buffer.append(1, (char)cp8);
                    cp8 = (uint8_t)((cp & 0x3f) | 0x80);
                    _value.buffer.append(1, (char)cp8);
                  } else if (cp >= 0xD800u && cp <= 0xDFFFu) {
                    // UTF-16 Surrogate pairs -- according to the UTF-8
                    // definition (RFC 3629) the high and low surrogate halves
                    // used by UTF-16 (U+D800 through U+DFFF) are not legal
                    // Unicode values, and the UTF-8 encoding of them is an
                    // invalid byte sequence. Instead of throwing an error, we
                    // substitute this character with the replacement character
                    // U+FFFD (UTF-8: EF,BF,BD).
                    _value.buffer.append("\xEF\xBF\xBD");
                    // 
                  } else {
                    // U+0800 - U+FFFF
                    uint8_t cp8 = (uint8_t)((cp >> 12) | 0xe0);
                    _value.buffer.append(1, (char)cp8);
                    cp8 = (uint8_t)(((cp >> 6) & 0x3f) | 0x80);
                    _value.buffer.append(1, (char)cp8);
                    cp8 = (uint8_t)((cp & 0x3f) | 0x80);
                    _value.buffer.append(1, (char)cp8);
                  }

                  break;
                }
                default:
                  _value.buffer.append(1, (char)b); break;
              }
              break;
            }

            case '"':
              goto after_initial_read_b;

            case 0:
              return setError(InvalidByte);

            default: {
              if (_value.buffered) {
                // TODO: Make this efficient by appending chunks between
                // boundaries instead of appending per-byte
                _value.buffer.append(1, (char)b);
              }
              break;
            }
          } // switch(b)
        } // while (!endOfInput())

        after_initial_read_b:
        if (b != '"') {
          return setError(UnterminatedString);
        }

        if (!_value.buffered) {
          _value.length = _input.offset - _value.offset - 1;
        }

        // is this a field name?
        while (!endOfInput()) {
          b = _input.bytes[_input.offset++];
          switch (b) {
            case ' ': case '\t': case '\r': case '\n': break;
            case ':': return setToken(FieldName);
            case ',': goto string_read_return_string;
            case ']': case '}': {
              --_input.offset; // rewind
              goto string_read_return_string;
            }
            case 0: return setError(InvalidByte);
            default: {
              // Expected a comma or a colon
              return setError(SyntaxError);
            }
          }
        }

        string_read_return_string:
        return setToken(jsont::String);
      }

      case ',': {
        if (_token == ObjectStart || _token == ArrayStart || _token == _Comma) {
          return setError(UnexpectedComma);
        }
        _token = _Comma;
        break;
      }

      default: {
        if (isdigit((int)b) || b == '+' || b == '-') {
          // We are reading a number
          _value.beginAtOffset(_input.offset-1);
          Token token = jsont::Integer;

          while (!endOfInput()) {
            b = _input.bytes[_input.offset++];
            switch (b) {
              case '0'...'9': break;
              case '.': token = jsont::Float; break;
              case 'E': case 'e': case '-': case '+': {
                if (token != jsont::Float) {
                  return setError(MalformedNumberLiteral);
                }
                break;
              }
              default: {
                if ( (_input.offset - _value.offset == 1) &&
                     (_input.bytes[_value.offset] == '-' || 
                      _input.bytes[_value.offset] == '+') ) {
                  return setError(MalformedNumberLiteral);
                }

                // rewind the byte that terminated this number literal
                --_input.offset;

                _value.length = _input.offset - _value.offset - 1;
                return setToken(token);
              }
            }
          }
          return setToken(End);
        } else {
          return setError(InvalidByte);
        }
      }
    }
  }

  return setToken(End);
}


enum {
  kUTF8ByteVerbatim = 0,
  kUTF8ByteEncode1, // "\u000x"
  kUTF8ByteEncode2, // "\u00xx"
};
#define V kUTF8ByteVerbatim
#define E1 kUTF8ByteEncode1
#define E2 kUTF8ByteEncode2
static const uint8_t kUTF8ByteTable[256] = {
  E1, E1, E1, E1, E1, E1, E1, E1, 'b', 't', 'n', E1, 'f', 'r', E1, E1, E2, E2,
  E2, E2, E2, E2, E2, E2, E2, E2, E2, E2, E2, E2, E2, E2, V, V, '"', V, V, V, V,
  V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V,
  V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V,
  V, '\\', V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V,
  V, V, V, V, V, V, V, V, V, V, V, E2, V, V, V, V, V, V, V, V, V, V, V, V, V, V,
  V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V,
  V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V,
  V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V,
  V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V, V,
  V, V, V, V, V, V, V, V, V, V
};
#undef V
#undef E1
#undef E2

// #ifndef __has_feature
//   #define __has_feature(x) 0
// #endif
// #if defined(__cplusplus) && __has_feature(cxx_static_assert)
//   #define JSONT_CONST_ASSERT(expr, error_msg) static_assert((expr), (error_msg))
// #elif __has_feature(c_static_assert)
//   #define JSONT_CONST_ASSERT(expr, error_msg) _Static_assert((expr), (error_msg))
// #else
//   #define JSONT_CONST_ASSERT(expr, error_msg) ((void)0)
// #endif

Builder& Builder::appendString(const uint8_t* v, size_t length, TextEncoding encoding) {
  reserve(length + 2);
  _buf[_size++] = '"';

  assert(encoding == UTF8TextEncoding /* Currently only UTF-8 is supported */);

  const uint8_t* end = v+length;
  while (v != end) {
    uint8_t s = kUTF8ByteTable[*v];
    switch (s) {
      case kUTF8ByteVerbatim:
        _buf[_size++] = *v;
        break;
      case kUTF8ByteEncode1: {
        assert(*v < 16);
        size_t remainingSize = end-v+1+5; // five additional bytes needed
        reserve(remainingSize);
        _buf[_size] = '\\';
        _buf[++_size] = 'u';
        _buf[++_size] = '0';
        _buf[++_size] = '0';
        _buf[++_size] = '0';
        _buf[++_size] = *v + (*v > 10 ? 55 : 48); // A-F : 0-9
        ++_size;
        assert(_size <= _capacity);
        break;
      }
      case kUTF8ByteEncode2: {
        // Note: *v is guaranteed to be within the set [16,32),127. This is
        // an affect of the kUTF8ByteTable lookup table and this code needs to
        // be revised if the lookup table adds or removes any kUTF8ByteEncode.
        assert((*v > 15 && *v < 32) || *v == 127);
        size_t remainingSize = end-v+1+5; // five additional bytes needed
        reserve(remainingSize);
        _buf[_size] = '\\';
        _buf[++_size] = 'u';
        _buf[++_size] = '0';
        _buf[++_size] = '0';
        uint8_t b1 = (*v & 0xf0) / 16;
        //uint8_t b1 = (*v & 0xf0) >> 4; // slightly faster but LE-specific
        uint8_t b2 = *v & 0x0f;
        _buf[++_size] = b1 + (b1 > 10 ? 55 : 48); // A-F : 0-9
        _buf[++_size] = b2 + (b2 > 10 ? 55 : 48); // A-F : 0-9
        ++_size;
        assert(_size <= _capacity);
        break;
      }
      default:
        // reverse solidus escape
        size_t remainingSize = end-v+1+1; // one additional byte needed
        reserve(remainingSize);
        _buf[_size++] = '\\';
        _buf[_size++] = s;
        assert(_size <= _capacity);
        break;
    }

    ++v;
  }

  _buf[_size++] = '"';
  assert(_size <= _capacity);
  return *this;
}

#if JSONT_CXX_RVALUE_REFS
  // Move constructor and assignment operator
  Builder::Builder(Builder&& other)
      : _buf(other._buf)
      , _capacity(other._capacity)
      , _size(other._size)
      , _state(other._state) {
    other._buf = 0;
  }

  Builder& Builder::operator=(Builder&& other) {
    _buf = other._buf; other._buf = 0;
    _capacity = other._capacity;
    _size = other._size;
    _state = other._state;
    return *this;
  }
#endif

Builder::Builder(const Builder& other)
    : _buf(0)
    , _capacity(other._capacity)
    , _size(other._size)
    , _state(other._state) {
  _buf = (char*)malloc(_capacity);
  memcpy((void*)_buf, (const void*)other._buf, _size);
}

Builder& Builder::operator=(const Builder& other) {
  _capacity = other._capacity;
  _size = other._size;
  _state = other._state;
  _buf = (char*)malloc(_capacity);
  memcpy((void*)_buf, (const void*)other._buf, _size);
  return *this;
}

} // namespace jsont
