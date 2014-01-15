#include "event2/buffer.h"
#include "event2/http.h"

#include <string>

namespace aos {
namespace http {

// An HTTP server that serves files from a directory using libevent.
// Also allows configuring certain URLs to be dynamically generated.
class HTTPServer {
 public:
  HTTPServer(const char *directory, uint16_t port);
  // Starts serving pages.
  // Might not clean up everything before returning.
  void Run();
 protected:
  template<class T> class MemberHandler {
   public:
    typedef void (T::*Handler)(evhttp_request *);
    struct Holder {
      T *self;
      Handler handler;
    };
    static void Call(evhttp_request *request, void *handler_in) {
      const Holder *const holder = static_cast<Holder *>(handler_in);
      AddStandardHeaders(request);
      ((holder->self)->*(holder->handler))(request);
    }
  };
  void AddPage(const std::string &path, void (*handler)(evhttp_request *, void *),
               void *data);
  template<class T> void AddPage(const std::string &path,
                                 typename MemberHandler<T>::Handler handler,
                                 T *self) {
    // have to put "typename" in, so the typedef makes it clearer
    typedef typename MemberHandler<T>::Holder HolderType;
    AddPage(path, MemberHandler<T>::Call, new HolderType{self, handler});
  }
  // This gets set up as the generic handler.
  // It can also be called separately to serve the file that the request is
  // requesting from the filesystem.
  void ServeFile(evhttp_request *request);
 private:
  // The directory where files to be served come from.
  const char *directory_;
  // The main libevent structure.
  event_base *const base_;
  // The libevent HTTP server handle.
  evhttp *const http_;
  static void AddStandardHeaders(evhttp_request *request);
  static void StaticServeFile(evhttp_request *request, void *self) {
    static_cast<HTTPServer *>(self)->ServeFile(request);
  }
};

}  // namespace http
}  // namespace aos
