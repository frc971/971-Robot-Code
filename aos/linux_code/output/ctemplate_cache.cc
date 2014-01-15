#include "aos/linux_code/output/ctemplate_cache.h"

#include "aos/linux_code/configuration.h"
#include "aos/common/once.h"

namespace aos {
namespace http {

namespace {
ctemplate::TemplateCache *CreateTemplateCache() {
  ctemplate::TemplateCache *r = new ctemplate::TemplateCache();

  r->SetTemplateRootDirectory(configuration::GetRootDirectory());

  return r;
}
}  // namespace
ctemplate::TemplateCache *get_template_cache() {
  static Once<ctemplate::TemplateCache> once(CreateTemplateCache);
  return once.Get();
}

}  // namespace http
}  // namespace aos
