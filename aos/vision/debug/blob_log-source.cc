#include "aos/vision/debug/debug_framework.h"

#include <gdk/gdk.h>
#include <gtk/gtk.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <functional>
#include <string>

#include "aos/vision/blob/codec.h"

namespace aos {
namespace vision {

namespace {

long GetFileSize(const std::string& filename) {
  struct stat stat_buf;
  int rc = stat(filename.c_str(), &stat_buf);
  return rc == 0 ? stat_buf.st_size : -1;
}

// Parses the blob-log file format.
// File format goes:
//
// Repeated:
//
// frame_length.
// timestamp.
// fmt.w
// fmt.h
// Encoded blob.
class InputFile {
 public:
  InputFile(const std::string &fname)
      : ifs_(fname, std::ifstream::in), len_(GetFileSize(fname)) {
    if (len_ <= 0) {
      LOG(FATAL, "File (%s) not found. Size (%d)\n", fname.c_str(), (int)len_);
    }
    // assert(len_ > 0);
    tmp_buf_.resize(len_, 0);
    ifs_.read(&tmp_buf_[0], len_);
    buf_ = &tmp_buf_[0];
  }

  bool ReadNext(BlobList *blob_list, ImageFormat *fmt, uint64_t *timestamp) {
    if (buf_ - &tmp_buf_[0] >= len_) return false;
    if (prev_ != nullptr) prev_frames_.emplace_back(prev_);
    prev_ = buf_;
    DoRead(blob_list, fmt, timestamp);
    return true;
  }

  bool ReadPrev(BlobList *blob_list, ImageFormat *fmt, uint64_t *timestamp) {
    if (prev_frames_.empty()) return false;
    buf_ = prev_frames_.back();
    prev_frames_.pop_back();
    buf_ += sizeof(uint32_t);
    DoRead(blob_list, fmt, timestamp);
    prev_ = nullptr;
    return true;
  }

 private:
  void DoRead(BlobList *blob_list, ImageFormat *fmt, uint64_t *timestamp) {
    buf_ += sizeof(uint32_t);
    *timestamp = Int64Codec::Read(&buf_);
    fmt->w = Int32Codec::Read(&buf_);
    fmt->h = Int32Codec::Read(&buf_);
    buf_ = ParseBlobList(blob_list, buf_);
  }
  std::vector<const char *> prev_frames_;
  const char *buf_;
  const char *prev_ = nullptr;
  std::ifstream ifs_;

  long len_;
  std::vector<char> tmp_buf_;
};

// A single parsed frame.
class BlobStreamFrame {
 public:
  BlobList blob_list;
  ImageFormat fmt;
  uint64_t timestamp;
  void ReadNext(InputFile *fin) {
    blob_list.clear();
    if (!fin->ReadNext(&blob_list, &fmt, &timestamp)) {
      exit(0);
      return;
    }
  }
  bool ReadPrev(InputFile *fin) {
    blob_list.clear();
    return fin->ReadPrev(&blob_list, &fmt, &timestamp);
  }
};

}  // namespace

// class for installing a lambda as a gtk timeout.
class TimeoutCallback {
 public:
  TimeoutCallback() {}

  void Reset(guint32 interval, std::function<bool()> callback) {
    Stop();
    callback_ = callback;
    timeout_key_ = g_timeout_add(interval, &TimeoutCallback::Callback, this);
  }
  void Stop() {
    if (callback_) {
      g_source_remove(timeout_key_);
    }
    callback_ = std::function<bool()>();
  }

 private:
  static gint Callback(void *self) {
    return reinterpret_cast<TimeoutCallback *>(self)->Callback();
  }
  gint Callback() {
    auto callback = callback_;
    if (!callback()) {
      return FALSE;
    }
    return TRUE;
  }
  gint timeout_key_;
  std::function<bool()> callback_;
};

class BlobLogImageSource : public ImageSource {
 public:
  void Init(const std::string &blob_log_filename,
            DebugFrameworkInterface *interface) override {
    interface_ = interface;
    image_source_.reset(new InputFile(blob_log_filename));

    // Tick 25 fps.
    // TODO(parker): Make this FPS configurable.
    cb_.Reset(1000 / 25, [this]() { return Tick(); });

    frame_.ReadNext(image_source_.get());
    interface_->NewBlobList(frame_.blob_list, frame_.fmt);
    interface_->InstallKeyPress([this](uint32_t keyval) {
      if (keyval == GDK_KEY_Left) {
        frame_.ReadPrev(image_source_.get());
        interface_->NewBlobList(frame_.blob_list, frame_.fmt);
      } else if (keyval == GDK_KEY_Right) {
        frame_.ReadNext(image_source_.get());
        interface_->NewBlobList(frame_.blob_list, frame_.fmt);
      } else {
        return;
      }
    });
  }

  bool Tick() {
    frame_.ReadNext(image_source_.get());
    interface_->NewBlobList(frame_.blob_list, frame_.fmt);
    return true;
  }

  const char *GetHelpMessage() override {
    return &R"(
    format_spec is the name of a file in blob list format.
    This viewer source will stream blobs from the log.
)"[1];
  }

 private:
  TimeoutCallback cb_;
  DebugFrameworkInterface *interface_ = nullptr;
  std::unique_ptr<InputFile> image_source_;
  BlobStreamFrame frame_;
};

REGISTER_IMAGE_SOURCE("blob_log", BlobLogImageSource);

}  // namespace vision
}  // namespace aos
