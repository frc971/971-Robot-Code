# JSON Tokenizer (jsont)

A minimal and portable JSON tokenizer written in standard C and C++ (two separate versions). Performs validating and highly efficient parsing suitable for reading JSON directly into custom data structures. There are no code dependencies — simply include `jsont.{h,hh,c,cc}` in your project.

Build and run unit tests:

    make

## Synopsis

C API:

```c
jsont_ctx_t* S = jsont_create(0);
jsont_reset(S, uint8_t* inbuf, size_t inbuf_len);
tok = jsont_next(S)
// branch on `tok` ...
V = jsont_*_value(S[, ...]);
jsont_destroy(S);
```

New C++ API:

```cc
jsont::Tokenizer S(const char* inbuf, size_t length);
jsont::Token token;
while ((token = S.next())) {
  if (token == jsont::Float) {
    printf("%g\n", S.floatValue());
  } ... else if (t == jsont::Error) {
    // handle error
    break;
  }
}
```

```cc
jsont::Builder json;
json.startObject()
    .fieldName("foo").value(123.45)
    .fieldName("bar").startArray()
      .value(678)
      .value("nine \"ten\"")
    .endArray()
  .endObject();
std::cout << json.toString() << std::endl;
// {"foo":123.45,"bar":[678,"nine \"ten\""]}
```

# API overview

See `jsont.h` and `jsont.hh` for a complete overview of the API, incuding more detailed documentation. Here's an overview:

## C++ API `namespace jsont`

- `Builder build()` — convenience builder factory

### class Tokenizer

Reads a sequence of bytes and produces tokens and values while doing so.

- `Tokenizer(const char* bytes, size_t length, TextEncoding encoding)` — initialize a new Tokenizer to read `bytes` of `length` in `encoding`
- `void reset(const char* bytes, size_t length, TextEncoding encoding)` — Reset the tokenizer, making it possible to reuse this parser so to avoid unnecessary memory allocation and deallocation.

#### Reading tokens

- `const Token& next() throw(Error)` — Read next token, possibly throwing an `Error`
- `const Token& current() const` — Access current token

#### Reading values

- `bool hasValue() const` — True if the current token has a value
- `size_t dataValue(const char const** bytes)` — Returns a slice of the input which represents the current value, or nothing (returns 0) if the current token has no value (e.g. start of an object).
- `std::string stringValue() const` — Returns a *copy* of the current string value.
- `double floatValue() const` — Returns the current value as a double-precision floating-point number.
- `int64_t intValue() const` — Returns the current value as a signed 64-bit integer.

#### Handling errors

- `ErrorCode error() const` — Returns the error code of the last error
- `const char* errorMessage() const` — Returns a human-readable message for the last error. Never returns NULL.

#### Acessing underlying input buffer

- `const char* inputBytes() const` — A pointer to the input data as passed to `reset` or the constructor.
- `size_t inputSize() const` — Total number of input bytes
- `size_t inputOffset() const` — The byte offset into input where the tokenizer is currently at. In the event of an error, this will point to the source of the error.

### enum Token

- `End` —           Input ended
- `ObjectStart` —   {
- `ObjectEnd` —     }
- `ArrayStart` —    [
- `ArrayEnd` —      ]
- `True` —          true
- `False` —         false
- `Null` —          null
- `Integer` —       number value without a fraction part (access as int64 through `Tokenizer::intValue()`)
- `Float` —         number value with a fraction part (access as double through `Tokenizer::floatValue()`)
- `String` —        string value (access value through `Tokenizer::stringValue()` et al)
- `FieldName` —     field name (access value through `Tokenizer::stringValue()` et al)
- `Error` —         an error occured (access error code through `Tokenizer::error()` et al)

### enum TextEncoding

- `UTF8TextEncoding` — Unicode UTF-8 text encoding

### enum Tokenizer::ErrorCode

- `UnspecifiedError` — Unspecified error
- `UnexpectedComma` — Unexpected comma
- `UnexpectedTrailingComma` — Unexpected trailing comma
- `InvalidByte` — Invalid input byte
- `PrematureEndOfInput` — Premature end of input
- `MalformedUnicodeEscapeSequence` — Malformed Unicode escape sequence
- `MalformedNumberLiteral` — Malformed number literal
- `UnterminatedString` — Unterminated string
- `SyntaxError` — Illegal JSON (syntax error)

### class Builder

Aids in building JSON, providing a final sequential byte buffer.

- `Builder()` — initialize a new builder with an empty backing buffer
- `Builder& startObject()` — Start an object (appends a `'{'` character to the backing buffer)
- `Builder& endObject()` — End an object (a `'}'` character)
- `Builder& startArray()` — Start an array (`'['`)
- `Builder& endArray()` — End an array (`']'`)
- `const void reset()` — Reset the builder to its neutral state. Note that the backing buffer is reused in this case.

#### Building

- `Builder& fieldName(const char* v, size_t length, TextEncoding encoding=UTF8TextEncoding)` — Adds a field name by copying `length` bytes from `v`.
- `Builder& fieldName(const std::string& name, TextEncoding encoding=UTF8TextEncoding)` — Adds a field name by copying `name`.
- `Builder& value(const char* v, size_t length, TextEncoding encoding=UTF8TextEncoding)` — Adds a string value by copying `length` bytes from `v` which content is encoded according to `encoding`.
- `Builder& value(const char* v)` — Adds a string value by copying `strlen(v)` bytes from c-string `v`. Uses the default encoding of `value(const char*,size_t,TextEncoding)`.
- `Builder& value(const std::string& v)`  — Adds a string value by copying `v`. Uses the default encoding of `value(const char*,size_t,TextEncoding)`.
- `Builder& value(double v)` — Adds a possibly fractional number
- `Builder& value(int64_t v)`, `void value(int v)`, `void value(unsigned int v)`, `void value(long v)` — Adds an integer number
- `Builder& value(bool v)` — Adds the "true" or "false" atom, depending on `v`
- `Builder& nullValue()` — Adds the "null" atom

#### Managing the result

- `size_t size() const` — Number of readable bytes at the pointer returned by `bytes()`
- `const char* bytes() const` — Pointer to the backing buffer, holding the resulting JSON.
- `std::string toString() const` — Return a `std::string` object holding a copy of the backing buffer, representing the JSON.
- `const char* seizeBytes(size_t& size_out)` — "Steal" the backing buffer. After this call, the caller is responsible for calling `free()` on the returned pointer. Returns NULL on failure. Sets the value of `size_out` to the number of readable bytes at the returned pointer. The builder will be reset and ready to use (which will act on a new backing buffer).

----

## C API

### Types

- `jsont_ctx_t` — A tokenizer context ("instance" in OOP lingo.)
- `jsont_tok_t` — A token type (see "Token types".)
- `jsont_err_t` — A user-configurable error type, which defaults to `const char*`.

### Managing a tokenizer context

- `jsont_ctx_t* jsont_create(void* user_data)` — Create a new JSON tokenizer context.
- `void jsont_destroy(jsont_ctx_t* ctx)` — Destroy a JSON tokenizer context.
- `void jsont_reset(jsont_ctx_t* ctx, const uint8_t* bytes, size_t length)` — Reset the tokenizer to parse the data pointed to by `bytes`.

### Dealing with tokens

- `jsont_tok_t jsont_next(jsont_ctx_t* ctx)` — Read and return the next token.
- `jsont_tok_t jsont_current(const jsont_ctx_t* ctx)` — Returns the current token (last token read by `jsont_next`).

### Accessing and comparing values

- `int64_t jsont_int_value(jsont_ctx_t* ctx)` — Returns the current integer value.
- `double jsont_float_value(jsont_ctx_t* ctx)` — Returns the current floating-point number value.
- `size_t jsont_data_value(jsont_ctx_t* ctx, const uint8_t** bytes)` — Returns a slice of the input which represents the current value.
- `char* jsont_strcpy_value(jsont_ctx_t* ctx)` — Retrieve a newly allocated c-string.
- `bool jsont_data_equals(jsont_ctx_t* ctx, const uint8_t* bytes, size_t length)` — Returns true if the current data value is equal to `bytes` of `length`
- `bool jsont_str_equals(jsont_ctx_t* ctx, const char* str)` — Returns true if the current data value is equal to c string `str`.

Note that the data is not parsed until you call one of these functions. This means that if you know that a value transferred as a string will fit in a 64-bit signed integer, it's completely valid to call `jsont_int_value` to parse the string as an integer.

### Miscellaneous

- `uint8_t jsont_current_byte(jsont_ctx_t* ctx)` — Get the last byte read.
- `size_t jsont_current_offset(jsont_ctx_t* ctx)` — Get the current offset of the last byte read.
- `jsont_err_t jsont_error_info(jsont_ctx_t* ctx)` — Get information on the last error.
- `void* jsont_user_data(const jsont_ctx_t* ctx)` — Returns the value passed to `jsont_create`

### Token types

- `JSONT_END` —            Input ended.
- `JSONT_ERR` —            Error. Retrieve details through `jsont_error_info`
- `JSONT_OBJECT_START` —   {
- `JSONT_OBJECT_END` —     }
- `JSONT_ARRAY_START` —    [
- `JSONT_ARRAY_END` —      ]
- `JSONT_TRUE` —           true
- `JSONT_FALSE` —          false
- `JSONT_NULL` —           null
- `JSONT_NUMBER_INT` —     number value without a fraction part (access through `jsont_int_value` or `jsont_float_value`)
- `JSONT_NUMBER_FLOAT` —   number value with a fraction part (access through `jsont_float_value`)
- `JSONT_STRING` —         string value (access through `jsont_data_value` or `jsont_strcpy_value`)
- `JSONT_FIELD_NAME` —     field name (access through `jsont_data_value` or `jsont_strcpy_value`)

## Further reading

- See `example*.c` for working sample programs.
- See `LICENSE` for the MIT-style license under which this project is licensed.
