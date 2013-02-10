#include <string.h>

#include "aos/aos_core.h"
#include "aos/atom_code/output/HTTPServer.h"
#include "aos/atom_code/output/evhttp_ctemplate_emitter.h"
#include "ctemplate/template.h"

#include "frc971/constants.h"

RegisterTemplateFilename(ROBOT_HTML, "robot.html.tpl");

namespace frc971 {

const char *const kPath = "/home/driver/robot_code/bin/";
//const char *const kPath = "/home/brians/Desktop/git_frc971/2012/trunk/src/frc971/output";

class CameraServer : public aos::http::HTTPServer {
 public:
  CameraServer() : HTTPServer(kPath, 8080), buf_(NULL) {
    AddPage<CameraServer>("/robot.html", &CameraServer::RobotHTML, this);
  }

 private:
  evbuffer *buf_;
  bool Setup(evhttp_request *request, const char *content_type) {
    if (evhttp_add_header(evhttp_request_get_output_headers(request),
                          "Content-Type", content_type) == -1) {
      LOG(WARNING, "adding Content-Type failed\n");
      evhttp_send_error(request, HTTP_INTERNAL, NULL);
      return false;
    }
    if (buf_ == NULL) buf_ = evbuffer_new();
    if (buf_ == NULL) {
      LOG(WARNING, "evbuffer_new() failed\n");
      evhttp_send_error(request, HTTP_INTERNAL, NULL);
      return false;
    }
    return true;
  }
  void RobotHTML(evhttp_request *request) {
    if (!Setup(request, "text/html")) return;

    ctemplate::TemplateDictionary dict(ROBOT_HTML);
    const char *host = evhttp_find_header(
        evhttp_request_get_input_headers(request), "Host");
    if (host == NULL) {
      evhttp_send_error(request, HTTP_BADREQUEST, "no Host header");
      return;
    }
    const char *separator = strchrnul(host, ':');
    size_t length = separator - host;
    // Don't include the last ':' (or the terminating '\0') or anything else
    // after it.
    dict.SetValue("HOST", ctemplate::TemplateString(host, length));

    int center;
    if (!constants::camera_center(&center)) {
      evhttp_send_error(request, HTTP_INTERNAL, NULL);
      return;
    }
    dict.SetIntValue("CENTER", center);

    aos::http::EvhttpCtemplateEmitter emitter(buf_);
    if (!ctemplate::ExpandTemplate(ROBOT_HTML, ctemplate::STRIP_WHITESPACE,
                                   &dict, &emitter)) {
      LOG(ERROR, "expanding the template failed\n");
      evhttp_send_error(request, HTTP_INTERNAL, NULL);
      return;
    }
    if (emitter.error()) {
      evhttp_send_error(request, HTTP_INTERNAL, NULL);
      return;
    }
    evhttp_send_reply(request, HTTP_OK, NULL, buf_);
  }
};

}  // namespace frc971

AOS_RUN_NRT(frc971::CameraServer)

