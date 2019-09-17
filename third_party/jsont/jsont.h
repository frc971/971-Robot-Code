// JSON Tokenizer. Copyright (c) 2012, Rasmus Andersson. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be
// found in the LICENSE file.
#ifndef JSONT_INCLUDED
#define JSONT_INCLUDED

#include <stdint.h>  // uint8_t, int64_t
#include <stdlib.h>  // size_t
#include <string.h>  // strlen
#include <stdbool.h> // bool

#ifndef _JSONT_IN_SOURCE
typedef struct jsont_ctx jsont_ctx_t;
typedef uint8_t jsont_tok_t;
#endif

#ifndef JSONT_ERRINFO_CUSTOM
#define jsont_err_t const char*
#endif

// Token types
enum {
  JSONT_END = 0,        // Input ended
  JSONT_ERR,            // Error

  JSONT_OBJECT_START,   // {
  JSONT_OBJECT_END,     // }
  
  JSONT_ARRAY_START,    // [
  JSONT_ARRAY_END,      // ]
  
  JSONT_TRUE,           // true
  JSONT_FALSE,          // false
  JSONT_NULL,           // null

  _JSONT_VALUES_START,
  JSONT_NUMBER_INT,     // number value without a fraction part
  JSONT_NUMBER_FLOAT,   // number value with a fraction part
  JSONT_STRING,         // string value
  JSONT_FIELD_NAME,     // field name
  _JSONT_VALUES_END,

  _JSONT_COMMA,
};

#ifdef __cplusplus
extern "C" {
#endif

// Create a new JSON tokenizer context. `user_data` can be anything and is
// accessible through `jsont_user_data`.
jsont_ctx_t* jsont_create(void* user_data);

// Destroy a JSON tokenizer context. This will free any internal data, except
// from the input buffer.
void jsont_destroy(jsont_ctx_t* ctx);

// Reset the tokenizer to parse the data pointed to by `bytes`. The tokenizer
// does NOT take ownership of `bytes`. This function can be used to recycle a
// tokenizer context, minimizing memory reallocation.
void jsont_reset(jsont_ctx_t* ctx, const uint8_t* bytes, size_t length);

// Read and return the next token. See `jsont_tok_t` enum for a list of
// possible return values and their meaning.
jsont_tok_t jsont_next(jsont_ctx_t* ctx);

// Returns the current token (last token read by `jsont_next`).
jsont_tok_t jsont_current(const jsont_ctx_t* ctx);

// Returns a slice of the input which represents the current value, or nothing
// (returns 0) if the current token has no value (e.g. start of an object).
size_t jsont_data_value(jsont_ctx_t* ctx, const uint8_t** bytes);

// Returns true if the current data value is equal to `bytes` of `length`
bool jsont_data_equals(jsont_ctx_t* ctx, const uint8_t* bytes, size_t length);

// Returns true if the current data value is equal to c string `str`
static inline bool jsont_str_equals(jsont_ctx_t* ctx, const char* str) {
  return jsont_data_equals(ctx, (const uint8_t*)str, strlen(str));
}

// Retrieve a newly allocated c-string. Similar to `jsont_data_value` but
// returns a newly allocated copy of the current value as a C string
// (terminated by a null byte). The calling code is responsible for calling
// `free()` on the returned value.
char* jsont_strcpy_value(jsont_ctx_t* ctx);

// Returns the current integer value.If the number is too large or too small,
// this function sets errno and returns INT64_MAX or INT64_MIN.
int64_t jsont_int_value(jsont_ctx_t* ctx);

// Returns the current floating-point number value. Sets errno and returns a
// value that isnan(N)==true on error.
double jsont_float_value(jsont_ctx_t* ctx);

// Get the last byte read. Suitable for debugging JSONT_ERR.
uint8_t jsont_current_byte(jsont_ctx_t* ctx);

// Get the current offset of the last byte read.
size_t jsont_current_offset(jsont_ctx_t* ctx);

// Get information on the last error (by default a printable text message).
// Returns NULL if no error has occured since a call to `jsont_reset`.
jsont_err_t jsont_error_info(jsont_ctx_t* ctx);

// Returns the value passed to `jsont_create`.
void* jsont_user_data(const jsont_ctx_t* ctx);

// ----------------- C++ -----------------
#ifdef __cplusplus
} // extern "C"
#endif

#endif // JSONT_INCLUDED
