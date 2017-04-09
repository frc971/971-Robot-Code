#include "aos/vision/debug/debug_framework.h"

#include <gdk/gdk.h>
#include <gtk/gtk.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <functional>
#include <string>

#include "aos/vision/blob/codec.h"
#include "aos/vision/blob/stream_view.h"
#include "aos/vision/debug/overlay.h"

namespace aos {
namespace vision {

namespace {

__attribute__((format(printf, 1, 2))) std::string SPrintf(const char *format,
                                                          ...) {
  va_list arglist;
  va_start(arglist, format);
  size_t count = vsnprintf(nullptr, 0, format, arglist);
  va_end(arglist);

  char out[count + 1];
  va_start(arglist, format);
  vsnprintf(out, count + 1, format, arglist);
  va_end(arglist);
  return std::string(&out[0], &out[count]);
}

long GetFileSize(const std::string &filename) {
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
    if (buf_ - &tmp_buf_[0] + sizeof(uint32_t) >= static_cast<size_t>(len_))
      return false;
    if (prev_ != nullptr) prev_frames_.emplace_back(prev_);
    prev_ = buf_;
    DoRead(blob_list, fmt, timestamp);
    return true;
  }

  bool ReadPrev(BlobList *blob_list, ImageFormat *fmt, uint64_t *timestamp) {
    if (prev_frames_.empty()) return false;
    buf_ = prev_frames_.back();
    prev_frames_.pop_back();
    DoRead(blob_list, fmt, timestamp);
    prev_ = nullptr;
    return true;
  }

 private:
  void DoRead(BlobList *blob_list, ImageFormat *fmt, uint64_t *timestamp) {
    uint32_t size_delta = Int32Codec::Read(&buf_);
    if (buf_ - &tmp_buf_[0] + size_delta > len_) {
      fprintf(stderr, "Corrupted last record.\n");
      exit(-1);
    }
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

class FrameStats {
 public:
  void UpdateStats(int64_t timestamp, bool has_target) {
    if (has_target != last_has_target_) {
      if (has_target && timestamp > last_timestamp_) {
        ++frame_count_;
      }
      if (!has_target && timestamp < last_timestamp_) {
        --frame_count_;
      }
    }
    if (has_target && first_frame_timestamp_ == -1) {
      first_frame_timestamp_ = timestamp;
    }

    last_timestamp_ = timestamp;
    last_has_target_ = has_target;
  }

  void SetStartTimestamp(int64_t start_timestamp) {
    start_timestamp_ = start_timestamp;
  }

  std::string Summary() const {
    return SPrintf(" frame_count: %ld\n time_since_first_target: %7.3f",
                   frame_count_,
                   (last_timestamp_ - GetStartTimestamp()) / 1.0e9);
  }

  int64_t GetStartTimestamp() const {
    if (first_frame_timestamp_ != -1) return first_frame_timestamp_;
    return start_timestamp_;
  }

 private:
  int64_t start_timestamp_;
  int64_t frame_count_ = 0;
  int64_t first_frame_timestamp_ = -1;
  int64_t last_timestamp_ = -1;
  bool last_has_target_ = 0;
};

// TODO: display this on the screen when they press help.
const char *kHudText2 = &R"(
commands:
 h - display this message.
 space - pause / unpause at normal speed (No extra mode).
 s - Skip forward fast to the next target.
 p - Skip backwards fast to the previous target.
 left_arrow - single-step backwards.
 right_arrow - single-step forwards.

)"[1];

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
    start_timestamp_ = frame_.timestamp;
    frame_stats_.SetStartTimestamp(start_timestamp_);

    interface_->NewBlobList(frame_.blob_list, frame_.fmt);
    interface_->InstallKeyPress([this](uint32_t keyval) {
      if (keyval == GDK_KEY_Left) {
        ReadPrevFrame();
        bool has_target = interface_->NewBlobList(frame_.blob_list, frame_.fmt);
        frame_stats_.UpdateStats(frame_.timestamp, has_target);
      } else if (keyval == GDK_KEY_Right) {
        ReadNextFrame();
        bool has_target = interface_->NewBlobList(frame_.blob_list, frame_.fmt);
        frame_stats_.UpdateStats(frame_.timestamp, has_target);
      } else if (keyval == GDK_KEY_space) {
        if (mode_ != PAUSED) {
          mode_ = PAUSED;
        } else {
          mode_ = NORMAL_MODE;
        }
      } else if (keyval == GDK_KEY_s) {
        controller_.reset(new FastForwardUntilFrameController(this));
        mode_ = FAST_MODE;
      } else if (keyval == GDK_KEY_p) {
        controller_.reset(new FastForwardUntilFrameController(this));
        mode_ = FAST_MODE_REV;
      } else {
        return;
      }
    });
    interface_->viewer()->AddOverlay(&overlay_);
    overlay_.draw_fn = [this](RenderInterface *cr, double w, double h) {
      (void)w;
      (void)h;
      cr->SetSourceRGB(1, 0, 0);
      auto text = SPrintf(" time: %7.3f\n",
                          (frame_.timestamp - start_timestamp_) / 1.0e9);
      text += frame_stats_.Summary();
      cr->Text(2, h - 100, 0, 0, text);
    };
  }

  bool Tick() {
    if (need_timestamp_print_) {
      fprintf(stderr, "time: %g\n",
              (frame_.timestamp - start_timestamp_) / 1.0e9);
      need_timestamp_print_ = false;
    }
    for (int i = 0; i < GetSpeed(); ++i) {
      if (Direction()) {
        ReadNextFrame();
      } else {
        ReadPrevFrame();
      }
      bool has_target =
          interface_->JustCheckForTarget(frame_.blob_list, frame_.fmt);
      frame_stats_.UpdateStats(frame_.timestamp, has_target);
      if (controller_) {
        controller_->NewFrame(has_target);
      }
      // Draw on the last frame:
      if (i + 1 >= GetSpeed()) {
        interface_->NewBlobList(frame_.blob_list, frame_.fmt);
      }
    }
    return true;
  }

  int GetSpeed() {
    if (mode_ == PAUSED) return 0;
    if (mode_ == NORMAL_MODE) return 1;
    if (mode_ == FAST_MODE || mode_ == FAST_MODE_REV) return 60;
    return 0;
  }

  bool Direction() {
    if (mode_ == FAST_MODE_REV) return false;
    return true;
  }

  void ReadNextFrame() {
    frame_.ReadNext(image_source_.get());
    need_timestamp_print_ = true;
  }

  void ReadPrevFrame() {
    frame_.ReadPrev(image_source_.get());
    need_timestamp_print_ = true;
  }

  const char *GetHelpMessage() override {
    return &R"(
    format_spec is the name of a file in blob list format.
    This viewer source will stream blobs from the log.
)"[1];
  }

 private:
  bool need_timestamp_print_ = true;
  uint64_t start_timestamp_ = 0;

  class Controller {
   public:
    virtual ~Controller() {}
    virtual void NewFrame(bool has_target) = 0;
  };

  class FastForwardUntilFrameController : public Controller {
   public:
    FastForwardUntilFrameController(BlobLogImageSource *proxy)
        : proxy_(proxy) {}

    void NewFrame(bool has_target) override {
      if (!has_target) inside_target = false;
      if (!inside_target && has_target) {
        proxy_->mode_ = PAUSED;
        proxy_->controller_.reset(nullptr);
      }
    }

    BlobLogImageSource *proxy_;
    bool inside_target = true;
  };

  std::unique_ptr<Controller> controller_;

  FrameStats frame_stats_;

  enum Mode {
    PAUSED,
    NORMAL_MODE,
    FAST_MODE,
    FAST_MODE_REV,
  };
  Mode mode_ = PAUSED;

  // LambdaOverlay text_overlay_;
  TimeoutCallback cb_;
  DebugFrameworkInterface *interface_ = nullptr;
  std::unique_ptr<InputFile> image_source_;
  BlobStreamFrame frame_;
  LambdaOverlay overlay_;
};

REGISTER_IMAGE_SOURCE("blob_log", BlobLogImageSource);

}  // namespace vision
}  // namespace aos
