// JSON Tokenizer. Copyright (c) 2012, Rasmus Andersson. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be
// found in the LICENSE file.
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <ctype.h> // isdigit
#include <errno.h>
#include <string.h>
#include <math.h>
#include <assert.h>

// Error info
#ifndef JSONT_ERRINFO_CUSTOM
#define jsont_err_t const char*
#define DEF_EM(NAME, msg) static jsont_err_t JSONT_ERRINFO_##NAME = msg
DEF_EM(STACK_SIZE, "Stack size limit exceeded");
DEF_EM(UNEXPECTED_OBJECT_END,
  "Unexpected end of object while not in an object");
DEF_EM(UNEXPECTED_ARRAY_END, "Unexpected end of array while not in an array");
DEF_EM(UNEXPECTED_COMMA, "Unexpected \",\"");
DEF_EM(UNEXPECTED_COLON, "Unexpected \":\"");
DEF_EM(UNEXPECTED, "Unexpected input");
DEF_EM(UNEXPECTED_UNICODE_SEQ, "Malformed unicode encoded sequence in string");
#undef DEF_EM
#endif

// Size of stack used for structures (in/out array and objects). This value
// is a balance between memory size of a ctx and how many levels deep the
// tokenizer can go.
#define _STRUCT_TYPE_STACK_SIZE 512
#define _VALUE_BUF_MIN_SIZE 64

static const uint8_t kHexValueTable[55] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, // 0-0
  -1, -1, -1, -1, -1, -1, -1,
  10, 11, 12, 13, 14, 15, // A-F
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1,
  10, 11, 12, 13, 14, 15 // a-f
};

typedef uint8_t jsont_tok_t;

typedef struct jsont_ctx {
  void* user_data;
  const uint8_t* input_buf;
  const uint8_t* input_buf_ptr;
  size_t input_len;
  const uint8_t* input_buf_value_start;
  const uint8_t* input_buf_value_end;
  struct {
    uint8_t* data;
    size_t size;
    size_t length;
    bool inuse;
  } value_buf;
  jsont_err_t error_info;
  jsont_tok_t curr_tok;
  size_t st_stack_size;
  size_t st_stack_len;
  jsont_tok_t st_stack[_STRUCT_TYPE_STACK_SIZE];
} jsont_ctx_t;

#define _JSONT_IN_SOURCE
#include <jsont.h>

unsigned long _hex_str_to_ul(const uint8_t* bytes, size_t len) {
  unsigned long value = 0;
  unsigned long cutoff = ULONG_MAX / 16;
  int cutoff_digit = (int)(ULONG_MAX - cutoff * 16);

  for (size_t i = 0; i != len; ++i) {
    uint8_t b = bytes[i];
    int digit = (b > '0'-1 && b < 'f'+1) ? kHexValueTable[b-'0'] : -1;
    if (b == 0xff || // bad digit
        (value > cutoff) || // overflow
        ((value == cutoff) && (digit > cutoff_digit)) ) {
      return ULONG_MAX;
    } else {
      value = (value * 16) + digit;
    }
  }

  return value;
}

jsont_ctx_t* jsont_create(void* user_data) {
  jsont_ctx_t* ctx = (jsont_ctx_t*)calloc(1, sizeof(jsont_ctx_t));
  ctx->user_data = user_data;
  ctx->st_stack_size = _STRUCT_TYPE_STACK_SIZE;
  return ctx;
}

void jsont_destroy(jsont_ctx_t* ctx) {
  if (ctx->value_buf.data != 0) {
    free(ctx->value_buf.data);
  }
  free(ctx);
}

void jsont_reset(jsont_ctx_t* ctx, const uint8_t* bytes, size_t length) {
  ctx->input_buf_ptr = ctx->input_buf = bytes;
  ctx->input_len = length;
  ctx->st_stack_len = 0;
  ctx->curr_tok = JSONT_END;
  ctx->input_buf_value_start = 0;
  ctx->input_buf_value_end = 0;
  ctx->value_buf.length = 0;
  ctx->value_buf.inuse = false;
  ctx->error_info = 0;
}

jsont_tok_t jsont_current(const jsont_ctx_t* ctx) {
  return ctx->curr_tok;
}

void* jsont_user_data(const jsont_ctx_t* ctx) {
  return ctx->user_data;
}

// Get the current/last byte read. Suitable for debugging JSONT_ERR
uint8_t jsont_current_byte(jsont_ctx_t* ctx) {
  return (ctx->input_buf_ptr == 0) ? 0 : *(ctx->input_buf_ptr-1);
}

size_t jsont_current_offset(jsont_ctx_t* ctx) {
  return ctx->input_buf_ptr - ctx->input_buf;
}

jsont_err_t jsont_error_info(jsont_ctx_t* ctx) {
  return ctx->error_info;
}

inline static bool _no_value(jsont_ctx_t* ctx) {
  return ctx->input_buf_value_start == 0
      || ctx->curr_tok < _JSONT_VALUES_START
      || ctx->curr_tok > _JSONT_VALUES_END;
}

inline static size_t _input_avail(jsont_ctx_t* ctx) {
  return ctx->input_len - (ctx->input_buf_ptr - ctx->input_buf);
}

inline static uint8_t _next_byte(jsont_ctx_t* ctx) {
  return (_input_avail(ctx) == 0) ? 0 : *(ctx->input_buf_ptr++);
}

inline static jsont_tok_t _st_stack_top(const jsont_ctx_t* ctx) {
  return (ctx->st_stack_len != 0) ? ctx->st_stack[ctx->st_stack_len-1]
                                  : JSONT_END;
}

size_t jsont_data_value(jsont_ctx_t* ctx, const uint8_t** bytes) {
  if (_no_value(ctx)) {
    return 0;
  } else {
    if (ctx->value_buf.inuse) {
      *bytes = ctx->value_buf.data;
      return ctx->value_buf.length;
    } else {
      *bytes = ctx->input_buf_value_start;
      return ctx->input_buf_value_end - ctx->input_buf_value_start;
    }
  }
}

bool jsont_data_equals(jsont_ctx_t* ctx, const uint8_t* bytes, size_t length) {
  if (ctx->value_buf.inuse) {
    return (ctx->value_buf.length == length) &&
      (memcmp((const void*)ctx->value_buf.data,
        (const void*)bytes, length) == 0);
  } else {
    return (ctx->input_buf_value_end - ctx->input_buf_value_start ==
            (ssize_t)length) &&
           (memcmp((const void *)ctx->input_buf_value_start,
                   (const void *)bytes, length) == 0);
  }
}

char* jsont_strcpy_value(jsont_ctx_t* ctx) {
  if (_no_value(ctx)) {
    return 0;
  } else {
    const uint8_t* bytes = 0;
    size_t len = jsont_data_value(ctx, &bytes);
    char* buf = (char*)malloc(len+1);
    if (memcpy((void*)buf, (const void*)bytes, len) != buf) {
      return 0;
    }
    buf[len] = 0;
    return buf;
  }
}

int64_t jsont_int_value(jsont_ctx_t* ctx) {
  if (_no_value(ctx)) {
    return INT64_MIN;
  }

  const uint8_t* start = 0;
  size_t len = jsont_data_value(ctx, &start);
  if (len == 0) {
    return INT64_MIN;
  }
  const uint8_t* end = start + len + 1;

  bool negative;
  uint8_t b = *start++;
  const int base = 10;

  if (b == '-') {
    negative = true;
    b = *start++;
    if (start == end) {
      errno = EINVAL;
      return INT64_MIN;
    }
  } else {
    negative = false;
    if (b == '+') {
      b = *start++;
      if (start == end) {
        errno = EINVAL;
        return INT64_MIN;
      }
    }
  }

  uint64_t acc = 0;
  int any = 0;
  uint64_t cutoff = negative
    ? (uint64_t)-(INT64_MIN + INT64_MAX) + INT64_MAX
    : INT64_MAX;
  int cutlim = cutoff % base;
  cutoff /= base;
  for ( ; start != end; b = *start++) {
    if (b >= '0' && b <= '9') b -= '0'; else break;
    if (any < 0 || acc > cutoff || (acc == cutoff && b > cutlim)) {
      any = -1;
    } else {
      any = 1;
      acc *= base;
      acc += b;
    }
  }

  if (any < 0) {
    acc = negative ? INT64_MIN : INT64_MAX;
    errno = ERANGE;
  } else if (!any) {
    errno = EINVAL;
    return INT64_MIN;
  } else if (negative) {
    acc = -acc;
  }

  return (int64_t)acc;
}

#ifdef NAN
  #define _JSONT_NAN NAN
#else
  #define _JSONT_NAN nan(0)
#endif

double jsont_float_value(jsont_ctx_t* ctx) {
  // Note: This might cause a segfault if the input is at the end, so we cause
  // an error if we try to read a float value while at the end of the input.
  if (_no_value(ctx) || _input_avail(ctx) == 0) {
    errno = EINVAL;
    return _JSONT_NAN;
  }

  const uint8_t* bytes = 0;
  size_t len = jsont_data_value(ctx, &bytes);
  if (len == 0) {
    return _JSONT_NAN;
  }
  return atof((const char*)bytes);
}

inline static jsont_tok_t _set_tok(jsont_ctx_t* ctx, jsont_tok_t tok) {
  ctx->curr_tok = tok;

  if (tok != JSONT_END) {
    if (tok == JSONT_OBJECT_START) {
      if (ctx->st_stack_len == ctx->st_stack_size) {
        ctx->error_info = JSONT_ERRINFO_STACK_SIZE;
        return ctx->curr_tok = JSONT_ERR; // TODO: Grow st_stack
      }
       ctx->st_stack[ctx->st_stack_len++] = JSONT_OBJECT_START;

    } else if (tok == JSONT_OBJECT_END) {
      if (_st_stack_top(ctx) != JSONT_OBJECT_START) {
        ctx->error_info = JSONT_ERRINFO_UNEXPECTED_OBJECT_END;
        return ctx->curr_tok = JSONT_ERR;
      }
      --ctx->st_stack_len;

    } else if (tok == JSONT_ARRAY_START) {
      if (ctx->st_stack_len == ctx->st_stack_size) {
        ctx->error_info = JSONT_ERRINFO_STACK_SIZE;
        return ctx->curr_tok = JSONT_ERR;
      }
       ctx->st_stack[ctx->st_stack_len++] = JSONT_ARRAY_START;

    } else if (tok == JSONT_ARRAY_END) {
      if (_st_stack_top(ctx) != JSONT_ARRAY_START) {
        ctx->error_info = JSONT_ERRINFO_UNEXPECTED_ARRAY_END;
        return ctx->curr_tok = JSONT_ERR;
      }
      --ctx->st_stack_len;
    }
  }

  return tok;
}
inline static void _rewind_one_byte(jsont_ctx_t* ctx) {
  --ctx->input_buf_ptr;
}
inline static void _rewind_bytes(jsont_ctx_t* ctx, size_t n) {
  ctx->input_buf_ptr -= n;
}
inline static void _skip_bytes(jsont_ctx_t* ctx, size_t n) {
  ctx->input_buf_ptr += n;
}
inline static uint8_t _read_atom(jsont_ctx_t* ctx, size_t slacklen,
                                 jsont_tok_t tok) {
  if (_input_avail(ctx) < slacklen) {
    // rewind and wait for buffer fill
    _rewind_one_byte(ctx);
    return _set_tok(ctx, JSONT_END);
  } else {
    _skip_bytes(ctx, slacklen); // e.g. "ull" after "n" or "alse" after "f"
    return _set_tok(ctx, tok);
  }
}
inline static bool _expects_field_name(jsont_ctx_t* ctx) {
  return (   ctx->curr_tok == JSONT_OBJECT_START
          || (   ctx->curr_tok == _JSONT_COMMA
              && _st_stack_top(ctx) == JSONT_OBJECT_START) );
}

static void _value_buf_append(jsont_ctx_t* ctx, const uint8_t* data, size_t len) {
  //printf("_value_buf_append(<ctx>, %p, %zu)\n", data, len);
  if (ctx->value_buf.size == 0) {
    assert(ctx->value_buf.data == 0);
    ctx->value_buf.length = len;
    ctx->value_buf.size = len * 2;
    if (ctx->value_buf.size < _VALUE_BUF_MIN_SIZE) {
      ctx->value_buf.size = _VALUE_BUF_MIN_SIZE;
    }
    ctx->value_buf.data = (uint8_t*)malloc(ctx->value_buf.size);
    if (len != 0) {
      memcpy(ctx->value_buf.data, data, len);
    }
  } else {
    if (ctx->value_buf.length + len > ctx->value_buf.size) {
      size_t new_size = ctx->value_buf.size + (len * 2);
      ctx->value_buf.data = realloc(ctx->value_buf.data, new_size);
      assert(ctx->value_buf.data != 0);
      ctx->value_buf.size = new_size;
    }
    memcpy(ctx->value_buf.data + ctx->value_buf.length, data, len);
    ctx->value_buf.length += len;
  }
  ctx->value_buf.inuse = true;
}

jsont_tok_t jsont_next(jsont_ctx_t* ctx) {
  //
  // { } [ ] n t f "
  //         | | | |
  //         | | | +- /[^"]*/ "
  //         | | +- a l s e
  //         | +- r u e
  //         +- u l l
  //
  while (1) {
    uint8_t b = _next_byte(ctx);
    switch (b) {
      case '{': return _set_tok(ctx, JSONT_OBJECT_START);
      case '}': return _set_tok(ctx, JSONT_OBJECT_END);
      case '[': return _set_tok(ctx, JSONT_ARRAY_START);
      case ']': return _set_tok(ctx, JSONT_ARRAY_END);
      case 'n': return _read_atom(ctx, 3, JSONT_NULL);
      case 't': return _read_atom(ctx, 3, JSONT_TRUE);
      case 'f': return _read_atom(ctx, 4, JSONT_FALSE);
      case '"': {
        ctx->input_buf_value_start = ctx->input_buf_ptr;
        ctx->value_buf.inuse = false;
        ctx->value_buf.length = 0;
        uint8_t prev_b = 0;
        while (1) {
          b = _next_byte(ctx);

          if (b == '\\') {
            if (prev_b == '\\') {
              // This is an actual '\'.
              assert(ctx->value_buf.inuse == true); // should be buffering
              _value_buf_append(ctx, ctx->input_buf_ptr-1, 1); // append "\"
            } else {
              // Okay, this is an escape prefix. Move to buffering value.
              if (ctx->value_buf.inuse == 0) {
                _value_buf_append(ctx,
                  ctx->input_buf_value_start,
                  // any data before the "\":
                  (ctx->input_buf_ptr-1 - ctx->input_buf_value_start) );
              }
            }
          } else {
            // Any byte except '\'

            if (prev_b == '\\') {
              // Currently just after an escape character
              assert(ctx->value_buf.inuse == true); // should be buffering

              // JSON specifies a few "magic" characters that have a different
              // meaning than their value:
              switch (b) {
              case 'b':
                _value_buf_append(ctx, (const uint8_t*)"\b", 1);
                break;
              case 'f':
                _value_buf_append(ctx, (const uint8_t*)"\f", 1);
                break;
              case 'n':
                _value_buf_append(ctx, (const uint8_t*)"\n", 1);
                break;
              case 'r':
                _value_buf_append(ctx, (const uint8_t*)"\r", 1);
                break;
              case 't':
                _value_buf_append(ctx, (const uint8_t*)"\t", 1);
                break;
              case 'u': {
                // 4 hex digits should follow
                if (_input_avail(ctx) < 4) {
                  _rewind_bytes(ctx,
                    ctx->input_buf_ptr - (ctx->input_buf_value_start-1));
                  return _set_tok(ctx, JSONT_END);
                }
                unsigned long utf16cp = _hex_str_to_ul(ctx->input_buf_ptr, 4);
                ctx->input_buf_ptr += 4;
                if (utf16cp == ULONG_MAX) {
                  ctx->error_info = JSONT_ERRINFO_UNEXPECTED_UNICODE_SEQ;
                  return _set_tok(ctx, JSONT_ERR);
                }

                uint32_t cp = (uint16_t)(0xffff & utf16cp);

                // Is lead surrogate?
                if (cp >= 0xd800u && cp <= 0xdbffu) {
                  // TODO: Implement pairs by reading another "\uHHHH"
                  ctx->error_info = JSONT_ERRINFO_UNEXPECTED_UNICODE_SEQ;
                  return _set_tok(ctx, JSONT_ERR);
                }

                // Append UTF-8 byte(s) representing the Unicode codepoint `cp`
                if (cp < 0x80) {
                  uint8_t cp8 = ((uint8_t)cp);
                  _value_buf_append(ctx, (const uint8_t*)&cp8, 1);
                } else if (cp < 0x800) {
                  uint8_t cp8 = (uint8_t)((cp >> 6) | 0xc0);
                  _value_buf_append(ctx, (const uint8_t*)&cp8, 1);
                  cp8 = (uint8_t)((cp & 0x3f) | 0x80);
                  _value_buf_append(ctx, (const uint8_t*)&cp8, 1);
                } else {
                  uint8_t cp8 = (uint8_t)((cp >> 12) | 0xe0);
                  _value_buf_append(ctx, (const uint8_t*)&cp8, 1);
                  cp8 = (uint8_t)(((cp >> 6) & 0x3f) | 0x80);
                  _value_buf_append(ctx, (const uint8_t*)&cp8, 1);
                  cp8 = (uint8_t)((cp & 0x3f) | 0x80);
                  _value_buf_append(ctx, (const uint8_t*)&cp8, 1);
                }

                break;
              }
              default: {
                _value_buf_append(ctx, &b, 1);
                break;
              }
              } // switch

            } else {
              // Previous character was NOT an escape character

              if (b == '"') {
                // Well, this marks the end of a string
                ctx->input_buf_value_end = ctx->input_buf_ptr-1;
                return _set_tok(ctx, _expects_field_name(ctx)
                  ? JSONT_FIELD_NAME : JSONT_STRING);
                break;
              } else if (b == 0) {
                // Input buffer ends in the middle of a string
                _rewind_bytes(ctx,
                  ctx->input_buf_ptr - (ctx->input_buf_value_start-1));
                return _set_tok(ctx, JSONT_END);
              } else {
                if (ctx->value_buf.inuse) {
                  _value_buf_append(ctx, &b, 1);
                }
              }
            }
          }

          prev_b = b;
        }
      }
      case ',':
        if (   ctx->curr_tok == JSONT_OBJECT_START
            || ctx->curr_tok == JSONT_ARRAY_START
            || ctx->curr_tok == JSONT_END
            || ctx->curr_tok == JSONT_ERR) {
          if (ctx->curr_tok != JSONT_ERR)
            ctx->error_info = JSONT_ERRINFO_UNEXPECTED_COMMA;
          return _set_tok(ctx, JSONT_ERR);
        }
        _set_tok(ctx, _JSONT_COMMA);
        // read next by simply letting the outer "while" do its thing
        break;

      case ':':
        if (ctx->curr_tok != JSONT_FIELD_NAME) {
          ctx->error_info = JSONT_ERRINFO_UNEXPECTED_COLON;
          return _set_tok(ctx, JSONT_ERR);
        }
        // let the outer "while" do its thing
        break;

      case ' ': case '\r': case '\n': case '\t':
        // ignore whitespace and let the outer "while" do its thing
        break;

      case 0:
        //printf("** %d\n", __LINE__);
        return _set_tok(ctx, JSONT_END);

      default:
        if (isdigit((int)b) || b == '+' || b == '-') {
          // We are reading a number
          ctx->input_buf_value_start = ctx->input_buf_ptr-1;
          //uint8_t prev_b = 0;
          bool is_float = false;
          while (1) {
            b = _next_byte(ctx);
            if (b == '.') {
              is_float = true;
            } else if (!isdigit((int)b)) {
              _rewind_one_byte(ctx);
              ctx->input_buf_value_end = ctx->input_buf_ptr;
              return _set_tok(ctx, is_float ? JSONT_NUMBER_FLOAT
                                            : JSONT_NUMBER_INT);
            } else if (b == 0) {
              // Input buffer ends before we know that the number-value ended
              _rewind_bytes(ctx, ctx->input_buf_ptr
                                 - (ctx->input_buf_value_start-1));
              return _set_tok(ctx, JSONT_END);
            }
          }
        }

        ctx->error_info = JSONT_ERRINFO_UNEXPECTED;
        return _set_tok(ctx, JSONT_ERR);
    }
  } // while (1)
}

