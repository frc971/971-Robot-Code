#include "aos/linux_code/output/evhttp_ctemplate_emitter.h"

#include "aos/common/logging/logging.h"

namespace aos {
namespace http {

void EvhttpCtemplateEmitter::Emit(const char *s, size_t slen) {
  if (error_) return;
  if (evbuffer_add(buf_, s, slen) != 0) {
    LOG(ERROR, "evbuffer_add(%p, %p, %zd) failed\n",
        buf_, s, slen);
    error_ = true;
  }
}

}  // namespace http
}  // namespace aos
