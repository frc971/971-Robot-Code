#include "aos/events/logging/s3_fetcher.h"

#include <aws/core/Aws.h>
#include <aws/s3/model/ListObjectsV2Request.h>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

// When we first start reading a log folder, we end up reading the first part of
// each file twice. We could speed this up by restructuring the API so all the
// downloads could be started in parallel, and the initial chunk of data cached.
// However, even though this initial part is slower than necessary,
// decompressing and sorting the main part of the log file is still the slowest
// part, and this implementation does parallelize downloads with that, so it's
// good enough for now.

namespace aos::logger {
namespace {

struct AwsAPIOwner {
  Aws::SDKOptions options;

  AwsAPIOwner() {
    options.httpOptions.installSigPipeHandler = true;
    Aws::InitAPI(options);
  }
  ~AwsAPIOwner() { Aws::ShutdownAPI(options); }
};

// If this doesn't fit the largest message, the per-request overhead very
// quickly dominates log reading time, because there's no processing in between
// reading all the pieces of a single message. Bigger takes more memory, but our
// logs aren't split across all that many files usually, so this can be fairly
// large without increasing memory requirements for log reading too much.
constexpr int kChunkSize = 10 * 1024 * 1024;

void InitAwsAPI() { static AwsAPIOwner api_owner; }

Aws::Client::ClientConfiguration MakeClientConfiguration() {
  InitAwsAPI();
  Aws::Client::ClientConfiguration config;
  config.region = Aws::Region::AWS_GLOBAL;
  return config;
}

struct ParsedRange {
  uint64_t start, end, total_size;
};

ParsedRange ParseRange(std::string_view string) {
  static constexpr std::string_view kBytes = "bytes ";
  CHECK(string.substr(0, kBytes.size()) == kBytes)
      << ": Invalid range: " << string;
  string = string.substr(kBytes.size());

  const size_t dash = string.find('-');
  CHECK(dash != string.npos) << ": Invalid range: " << string;
  const size_t slash = string.find('/');
  CHECK(slash != string.npos) << ": Invalid range: " << string;

  ParsedRange result;
  const std::string_view start_string = string.substr(0, dash);
  CHECK(absl::SimpleAtoi(start_string, &result.start))
      << ": failed to parse " << start_string << " from " << string;
  const std::string_view end_string = string.substr(dash + 1, slash - dash - 1);
  CHECK(absl::SimpleAtoi(end_string, &result.end))
      << ": failed to parse " << end_string << " from " << string;
  const std::string_view total_string = string.substr(slash + 1);
  CHECK(absl::SimpleAtoi(total_string, &result.total_size))
      << ": failed to parse " << total_string << " from " << string;
  return result;
}

}  // namespace

ObjectName ParseUrl(std::string_view url) {
  static constexpr std::string_view kS3 = "s3://";
  if (url.substr(0, kS3.size()) != kS3) {
    LOG(FATAL) << "Not an S3 URL: " << url;
  }
  url = url.substr(kS3.size());
  const size_t slash = url.find('/');
  CHECK(slash != url.npos) << ": Invalid S3 URL: " << url;
  ObjectName result;
  result.bucket = url.substr(0, slash);
  result.key = url.substr(slash + 1);
  return result;
}

// This client is thread-safe, so it should be used globally. Destroying it can
// take a while to shut down all the threads.
Aws::S3::S3Client &GetS3Client() {
  static Aws::S3::S3Client result(MakeClientConfiguration());
  return result;
}

S3Fetcher::S3Fetcher(std::string_view url) : url_(url) {
  VLOG(1) << "opening " << url;
  // Start the initial request now.
  StartRequest();
}

size_t S3Fetcher::Read(uint8_t *begin, uint8_t *end) {
  VLOG(1) << "looking to read " << (end - begin);
  size_t total_read = 0;

  while (true) {
    // First copy any data we already have.
    const size_t current_size =
        std::min<size_t>(current_chunk_.size(), end - begin - total_read);
    memcpy(begin + total_read, current_chunk_.data(), current_size);
    total_read += current_size;
    current_chunk_.erase_front(current_size);
    if (static_cast<ssize_t>(total_read) == end - begin) {
      VLOG(1) << "Got all " << total_read;
      // Got all of what the caller wants, done now.
      return total_read;
    }
    CHECK_EQ(current_chunk_.size(), 0u)
        << ": Should have already copied this data out";
    if (end_of_object_) {
      VLOG(1) << "At end after " << total_read;
      // Nothing more to read.
      return total_read;
    }

    // Read data from the last request.
    CHECK(get_next_chunk_.valid()) << ": Should have a request started already";
    Aws::S3::Model::GetObjectOutcome get_outcome = get_next_chunk_.get();
    if (!get_outcome.IsSuccess()) {
      if (next_byte_to_request_ == 0 &&
          get_outcome.GetError().GetResponseCode() ==
              Aws::Http::HttpResponseCode::REQUESTED_RANGE_NOT_SATISFIABLE) {
        VLOG(1) << "At beginning of empty file";
        // This is what happens with an empty file.
        // TODO(Brian): Do a List operation to verify it's actually empty?
        CHECK_EQ(0u, total_read);
        end_of_object_ = true;
        return 0;
      }
      LOG(FATAL) << ": GET for " << url_
                 << " failed: " << get_outcome.GetError();
    }
    const ParsedRange content_range =
        ParseRange(get_outcome.GetResult().GetContentRange());
    const uint64_t content_bytes = content_range.end - content_range.start + 1;
    CHECK_EQ(content_range.start, next_byte_to_request_);
    next_byte_to_request_ += kChunkSize;

    auto &stream = get_outcome.GetResult().GetBody();
    current_chunk_.resize(content_bytes);
    stream.read(reinterpret_cast<char *>(current_chunk_.data()), content_bytes);
    const size_t stream_read = stream.gcount();
    VLOG(1) << "got " << stream_read << " from "
            << get_outcome.GetResult().GetContentRange();
    CHECK_EQ(stream_read, content_bytes);
    if (content_range.end + 1 == content_range.total_size) {
      end_of_object_ = true;
      continue;
    }

    // Kick off the next request.
    StartRequest();
  }

  return total_read;
}

void S3Fetcher::StartRequest() {
  Aws::S3::Model::GetObjectRequest get_request;
  const ObjectName object_name = ParseUrl(url_);
  get_request.SetBucket(object_name.bucket);
  get_request.SetKey(object_name.key);
  const uint64_t last_byte_to_request = next_byte_to_request_ + kChunkSize;
  get_request.SetRange(absl::StrCat("bytes=", next_byte_to_request_, "-",
                                    last_byte_to_request - 1));
  VLOG(1) << "request for " << next_byte_to_request_ << "-"
          << last_byte_to_request << ": " << get_request.GetRange();
  get_next_chunk_ = GetS3Client().GetObjectCallable(get_request);
}

std::vector<std::string> ListS3Objects(std::string_view url) {
  Aws::S3::Model::ListObjectsV2Request list_request;
  const ObjectName object_name = ParseUrl(url);
  list_request.SetBucket(object_name.bucket);
  list_request.SetPrefix(object_name.key);
  Aws::S3::Model::ListObjectsV2Outcome list_outcome =
      GetS3Client().ListObjectsV2(list_request);
  std::vector<std::string> result;
  while (true) {
    CHECK(list_outcome.IsSuccess()) << ": Listing objects for " << url
                                    << " failed: " << list_outcome.GetError();
    auto &list_result = list_outcome.GetResult();
    for (const Aws::S3::Model::Object &object : list_result.GetContents()) {
      result.push_back(absl::StrCat("s3://", list_outcome.GetResult().GetName(),
                                    "/", object.GetKey()));
      VLOG(2) << "got " << result.back();
    }
    if (!list_result.GetIsTruncated()) {
      break;
    }
    list_request.SetContinuationToken(list_result.GetNextContinuationToken());
    list_outcome = GetS3Client().ListObjectsV2(list_request);
  }
  return result;
}

}  // namespace aos::logger
