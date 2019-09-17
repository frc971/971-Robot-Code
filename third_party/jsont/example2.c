//
// This is an example of parsing and building strict documents into C structs.
//
// The general approach is that each object type has a struct type and a
// builder function. The struct type has members which represents its
// properties. The builder function is more intresting: It takes a tokenizer
// state and a struct instance. The builder function then reads each field
// name from the tokenizer and calls other builder functions (this is how this
// parser does flow control), and eventually stores the values into the struct
// instance.
//
#include <jsont.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// A simple array type
typedef struct my_array {
  size_t size;
  size_t count;
  void** items;
} my_array_t;

// Represents a user object
typedef struct my_user {
  const char* id;
  const char* name;
} my_user_t;

// Represents a response from our imaginary service
typedef struct my_response {
  int64_t timestamp;
  const char* viewer_id;
  my_array_t users;
} my_response_t;

// A helper macro for allocating a new struct instance
#define MY_NEW(T) (T*)malloc(sizeof(T))

// Some helper macros for dealing with growing arrays
#define MY_ARRAY_ALLOC(A, _size) do {\
    (A).items = (void*)malloc(sizeof(void*)*_size); \
    (A).count = 0; \
    (A).size = _size; \
  } while(0)
#define MY_ARRAY_RESIZE(A, _size) do {\
    (A).items = (void*)realloc((A).items, sizeof(void*)*_size); \
    (A).size = _size; \
  } while(0)
#define MY_ARRAY_APPEND(A, item) (A).items[(A).count++] = (void*)(item)
#define MY_NEXT_EXPECT(S, TOKTYPE) do { \
  if ((tok = jsont_next(S)) != TOKTYPE) { \
    printf("Error: Builder expected token " #TOKTYPE " (%d)\n", __LINE__); \
    return false; \
  }} while (0)

// Builder function for user objects
bool my_user_build(jsont_ctx_t* S, my_user_t* obj) {
  jsont_tok_t tok = jsont_current(S);
  if (tok != JSONT_OBJECT_START) return false;
  
  // for each field
  while ((tok = jsont_next(S)) == JSONT_FIELD_NAME) {
    const uint8_t* fieldname = 0;
    size_t len = jsont_data_value(S, &fieldname);

    if (memcmp("id", fieldname, len) == 0) {
      MY_NEXT_EXPECT(S, JSONT_STRING);
      obj->id = jsont_strcpy_value(S);
    
    } else if (memcmp("name", fieldname, len) == 0) {
      MY_NEXT_EXPECT(S, JSONT_STRING);
      obj->name = jsont_strcpy_value(S);

    } else {
      printf("%s: Unexpected field: \"%.*s\"\n", __FUNCTION__,
        (int)len, (const char*)fieldname);
      return false;
    }
  }

  return true;
}

// Builder function for response objects
bool my_response_build(jsont_ctx_t* S, my_response_t* obj) {
  jsont_tok_t tok = jsont_current(S);
  if (tok != JSONT_OBJECT_START) return false;

  // for each field
  while ((tok = jsont_next(S)) == JSONT_FIELD_NAME) {
    const uint8_t* fieldname = 0;
    size_t len = jsont_data_value(S, &fieldname);

    if (memcmp("timestamp", fieldname, len) == 0) {
      MY_NEXT_EXPECT(S, JSONT_NUMBER_INT);
      obj->timestamp = jsont_int_value(S);

    } else if (memcmp("viewer_id", fieldname, len) == 0) {
      MY_NEXT_EXPECT(S, JSONT_STRING);
      obj->viewer_id = jsont_strcpy_value(S);

    } else if (memcmp("users", fieldname, len) == 0) {
      MY_NEXT_EXPECT(S, JSONT_ARRAY_START);
      MY_ARRAY_ALLOC(obj->users, 10);

      // for each user object
      while ((tok = jsont_next(S)) == JSONT_OBJECT_START) {
        if (obj->users.count == obj->users.size)
          MY_ARRAY_RESIZE(obj->users, obj->users.size * 2);
        my_user_t* user = MY_NEW(my_user_t);
        if (!my_user_build(S, user))
          return false;
        MY_ARRAY_APPEND(obj->users, user);
      }
    } else {
      printf("%s: Unexpected field: \"%.*s\"\n", __FUNCTION__,
        (int)len, (const char*)fieldname);
      return false;
    }
  }
  
  return true;
}

// Our simple response parser entry point. Returns NULL on error.
my_response_t* my_parse_response(jsont_ctx_t* S) {
if (jsont_next(S) != JSONT_OBJECT_START) {
    printf("Expected JSON input to start with an object.\n");
    return 0;
  }
  my_response_t* rsp = MY_NEW(my_response_t);
  if (!my_response_build(S, rsp)) {
    free(rsp);
    return 0;
  }
  return rsp;
}

int main(int argc, const char** argv) {
  // Create a new reusable tokenizer
  jsont_ctx_t* S = jsont_create(0);

  // Sample "response" data
  const char* inbuf = "{"
    "\"viewer_id\": \"abc123\","
    "\"timestamp\": 1234567890,"
    "\"users\":["
      "{\"name\": \"John Smith\", \"id\": \"12c39a\"},\n"
      "{\"name\": \"John Doe\",   \"id\": \"01dk2\"},\n"
      "{\"name\": \"Kate Smith\", \"id\": \"apru1\"},\n"
      "{\"name\": \"Rebecca Doe\",\"id\": \"aRm26\"}\n"
    "]"
  "}";

  // Parse the sample "response" data
  jsont_reset(S, (const uint8_t*)inbuf, strlen(inbuf));
  my_response_t* rsp = my_parse_response(S);

  // Epic success?
  if (rsp) {
    printf("Built response structure.\n");
    printf("rsp->users.items[2]->name => \"%s\"\n",
      ((my_user_t*)rsp->users.items[2])->name );

  } else {
    printf("Failed to build response structure.\n");
    if (jsont_error_info(S) != 0) {
      fprintf(stderr, "Error: %s ('%c' at offset %lu)\n",
              jsont_error_info(S),
              (char)jsont_current_byte(S),
              (unsigned long)jsont_current_offset(S));
    }
    // Exit with error. Note: In a real application, you should call
    // `jsont_destroy` on the reusable tokenizer when done with it. Here we
    // just exit the program.
    return 1;
  }

  // Destroy our reusable tokenizer and exit
  jsont_destroy(S);
  return 0;
}
