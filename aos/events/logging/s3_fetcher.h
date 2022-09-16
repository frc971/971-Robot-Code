
#ifndef AOS_EVENTS_LOGGING_S3_FETCHER_H_
#define AOS_EVENTS_LOGGING_S3_FETCHER_H_

#include <aws/s3/S3Client.h>
#include <aws/s3/model/GetObjectRequest.h>

#include <future>
#include <string_view>

#include "aos/containers/resizeable_buffer.h"
#include "aos/events/logging/buffer_encoder.h"

namespace aos::logger {

// Fetches data from an S3 URL.
class S3Fetcher final : public DataDecoder {
 public:
  explicit S3Fetcher(std::string_view url);
  S3Fetcher(const S3Fetcher &) = delete;
  S3Fetcher &operator=(const S3Fetcher &) = delete;

  size_t Read(uint8_t *begin, uint8_t *end) final;
  std::string_view filename() const final { return url_; }

 private:
  const std::string url_;

  // The current chunk we're reading from. Empty if there is no current chunk or
  // we've read all of it.
  ResizeableBuffer current_chunk_;
  // If valid, the next chunk which we've triggered to be retrieved in the
  // background.
  std::future<Aws::S3::Model::GetObjectOutcome> get_next_chunk_;

  // The next byte index we're going to request. This means we've already made
  // requests for all prior bytes, but not necessarily received them.
  uint64_t next_byte_to_request_ = 0;

  // Set once we've received data for the end of the object. Some of it may
  // still be in current_chunk_ though.
  bool end_of_object_ = false;

  // Kicks off a request for the next chunk.
  void StartRequest();
};

Aws::S3::S3Client& GetS3Client();

struct ObjectName {
  std::string bucket, key;
};

ObjectName ParseUrl(std::string_view url);

// Does an S3 object listing with the given URL prefix. Returns the URLs for all
// the objects under it.
std::vector<std::string> ListS3Objects(std::string_view url);

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_S3_FETCHER_H_
