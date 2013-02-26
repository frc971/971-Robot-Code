#include "ctemplate/template_cache.h"

namespace aos {
namespace http {

// Retrieves the cache used by all of the aos functions etc.
// This cache will have its root directory set to the directory where the
// executable is running from.
ctemplate::TemplateCache *get_template_cache();

}  // namespace http
}  // namespace aos
