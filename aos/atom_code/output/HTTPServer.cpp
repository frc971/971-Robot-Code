#include "aos/atom_code/output/HTTPServer.h"

#include <inttypes.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include <memory>

#include "event2/event.h"

#include "aos/common/scoped_fd.h"
#include "aos/common/unique_malloc_ptr.h"

namespace aos {
namespace http {

HTTPServer::HTTPServer(const char *directory, uint16_t port) :
    directory_(directory), base_(event_base_new()), http_(evhttp_new(base_)) {
  if (base_ == NULL) {
    LOG(FATAL, "couldn't create an event_base\n");
  }
  if (http_ == NULL) {
    LOG(FATAL, "couldn't create an evhttp\n");
  }
  if (evhttp_bind_socket(http_, "0.0.0.0", port) != 0) {
    LOG(FATAL, "evhttp_bind_socket(%p, \"0.0.0.0\", %"PRIu16") failed\n",
        http_, port);
  }
  evhttp_set_gencb(http_, StaticServeFile, this);
}

void HTTPServer::AddPage(const std::string &path,
                         void (*handler)(evhttp_request *, void *), void *data) {
  switch (evhttp_set_cb(http_, path.c_str(), handler, data)) {
    case 0:
      LOG(DEBUG, "set callback handler for '%s'\n", path.c_str());
      break;
    case -1:
      LOG(INFO, "changed callback handler for '%s'\n", path.c_str());
      break;
    default:
      LOG(WARNING, "evhttp_set_cb(%p, %s, %p, %p) failed\n", http_, path.c_str(),
          handler, data);
      break;
  }
}

void HTTPServer::AddStandardHeaders(evhttp_request *request) {
  if (evhttp_add_header(evhttp_request_get_output_headers(request),
                        "Server", "aos::HTTPServer/0.0") == -1) {
    LOG(WARNING, "adding Server header failed\n");
  }
}

namespace {
// All of these functions return false, NULL, or -1 if they fail (and send back
// an error).

// Returns the path of the file that is being requested.
const char *GetPath(evhttp_request *request) {
  // Docs are unclear whether this needs freeing, but it looks like it just
  // returns an internal field of the request.
  // Running valgrind with no freeing of uri or path doesn't report anything
  // related to this code.
  const evhttp_uri *uri = evhttp_request_get_evhttp_uri(request);
  const char *path = evhttp_uri_get_path(uri);
  if (path == NULL) {
    evhttp_send_error(request, HTTP_BADREQUEST, "need a path");
    return NULL;
  }
  if (strstr(path, "..") != NULL) {
    evhttp_send_error(request, HTTP_NOTFOUND, "no .. allowed!!");
    return NULL;
  }
  return path;
}
// Returns an fd open for reading for the file at "directory/path".
int OpenFile(evhttp_request *request, const char *path,
                   const char *directory) {
  char *temp;
  if (asprintf(&temp, "%s/%s", directory, path) == -1) {
    LOG(WARNING, "asprintf(%p, \"%%s/%%s\", %p, %p) failed with %d: %s\n",
        &temp, directory, path, errno, strerror(errno));
    evhttp_send_error(request, HTTP_INTERNAL, NULL);
    return -1;
  }
  const unique_c_ptr<char> filename(temp);
  ScopedFD file(open(filename.get(), O_RDONLY));
  if (!file) {
    if (errno == ENOENT) {
      evhttp_send_error(request, HTTP_NOTFOUND, NULL);
      return -1;
    }
    LOG(ERROR, "open('%s', 0) failed with %d: %s\n", filename.get(),
        errno, strerror(errno));
    evhttp_send_error(request, HTTP_INTERNAL, NULL);
    return -1;
  }
  return file.release();
}
// Returns the size of the file specified by the given fd.
off_t GetSize(int file) {
  struct stat info;
  if (fstat(file, &info) == -1) {
    LOG(ERROR, "stat(%d, %p) failed with %d: %s\n", file, &info,
        errno, strerror(errno));
    return -1;
  }
  return info.st_size;
}
bool SendFileResponse(evhttp_request *request, int file_num) {
  ScopedFD file(file_num);
  const off_t size = GetSize(file.get());
  if (size == -1) {
    evhttp_send_error(request, HTTP_INTERNAL, NULL);
    return false;
  }
  evbuffer *const buf = evhttp_request_get_output_buffer(request);
  if (evbuffer_add_file(buf, file.get(), 0, size) == -1) {
    LOG(WARNING, "evbuffer_add_file(%p, %d, 0, %jd) failed\n", buf,
        file.get(), static_cast<intmax_t>(size));
    evhttp_send_error(request, HTTP_INTERNAL, NULL);
    return false;
  } else {
    // it succeeded, so evhttp takes ownership
    file.release();
  }
  evhttp_send_reply(request, HTTP_OK, NULL, NULL);
  return true;
}

}  // namespace
void HTTPServer::ServeFile(evhttp_request *request) {
  AddStandardHeaders(request);

  const char *path = GetPath(request);
  if (path == NULL) return;

  ScopedFD file(OpenFile(request, path, directory_));
  if (!file) return;

  if (!SendFileResponse(request, file.release())) return;
}

void HTTPServer::Run() {
  event_base_dispatch(base_);
  LOG(FATAL, "event_base_dispatch returned\n");
}

}  // namespace http
}  // namespace aos
