#ifndef AOS_ATOM_CODE_OUTPUT_EVHTTP_CTEMPLATE_EMITTER_H_
#define AOS_ATOM_CODE_OUTPUT_EVHTTP_CTEMPLATE_EMITTER_H_

#include <string.h>

#include "event2/buffer.h"
#include "ctemplate/template_emitter.h"

namespace aos {
namespace http {

// Writes everything directly into an evbuffer*.
// Handles errors by refusing to write anything else into the buffer and storing
// the state (which can be retrieved with error()).
class EvhttpCtemplateEmitter : public ctemplate::ExpandEmitter {
 public:
  EvhttpCtemplateEmitter(evbuffer *buf) : buf_(buf), error_(false) {}
  virtual void Emit(char c) { Emit(&c, 1); };
  virtual void Emit(const std::string& s) { Emit(s.data(), s.size()); };
  virtual void Emit(const char* s) { Emit(s, strlen(s)); }
  virtual void Emit(const char* s, size_t slen);
  // Retrieves whether or not there has been an error. If true, the error will
  // already have been logged.
  bool error() { return error_; }

 private:
  evbuffer *const buf_;
  bool error_;
};

}  // namespace http
}  // namespace aos

#endif  // AOS_ATOM_CODE_OUTPUT_EVHTTP_CTEMPLATE_EMITTER_H_
