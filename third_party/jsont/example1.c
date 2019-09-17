//
// This is a simple example of running the tokenizer, outputting information
// to stdout about what tokens we get and their values.
//
#include <jsont.h>
#include <stdio.h>
#include <string.h>

static const char* _tok_name(jsont_tok_t tok);

int main(int argc, const char** argv) {
  // Create a new reusable tokenizer
  jsont_ctx_t* S = jsont_create(0);

  // Sample input
  const char* inbuf = "{\"Ape\":123,\"Bro\":[400192,\"51\",true, false, null,"
                      " -67,\r\n\t 6.123]}";

  // Reset the parser with a pointer to our sample input
  jsont_reset(S, (const uint8_t*)inbuf, strlen(inbuf));

  // Read each token
  jsont_tok_t tok;
  printf("Token        | Value\n"
         "-------------|----------------------------------------\n");
  while ( (tok = jsont_next(S)) != JSONT_END && tok != JSONT_ERR) {
    printf("%-12s |", _tok_name(tok));

    // If the token has a value, also print its value
    if (tok == JSONT_STRING || tok == JSONT_FIELD_NAME) {
      const uint8_t* bytes = 0;
      size_t len = jsont_data_value(S, &bytes);
      if (len != 0)
        printf(" '%.*s'", (int)len, (const char*)bytes);
    } else if (tok == JSONT_NUMBER_INT) {
      printf(" %lld", jsont_int_value(S));
    } else if (tok == JSONT_NUMBER_FLOAT) {
      printf(" %f", jsont_float_value(S));
    }
    
    printf("\n");
  }

  // If we got an error, print some useful information and exit with 1
  if (tok == JSONT_ERR) {
    fprintf(stderr, "Error: %s ('%c' at offset %lu)\n",
            jsont_error_info(S),
            (char)jsont_current_byte(S),
            (unsigned long)jsont_current_offset(S));
    return 1;
  }

  // Destroy our reusable tokenizer and exit
  jsont_destroy(S);
  return 0;
}

// Utility to get a printable name for a token
static const char* _tok_name(jsont_tok_t tok) {
  switch (tok) {
    case JSONT_END:           return "END";
    case JSONT_ERR:           return "ERR";
    case JSONT_OBJECT_START:  return "OBJECT_START";
    case JSONT_OBJECT_END:    return "OBJECT_END";
    case JSONT_ARRAY_START:   return "ARRAY_START";
    case JSONT_ARRAY_END:     return "ARRAY_END";
    case JSONT_TRUE:          return "TRUE";
    case JSONT_FALSE:         return "FALSE";
    case JSONT_NULL:          return "NULL";
    case JSONT_NUMBER_INT:    return "NUMBER_INT";
    case JSONT_NUMBER_FLOAT:  return "NUMBER_FLOAT";
    case JSONT_STRING:        return "STRING";
    case JSONT_FIELD_NAME:    return "FIELD_NAME";
    default:                  return "?";
  }
}
