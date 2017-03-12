#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <memory>
#include <endian.h>
#include <sys/stat.h>
#include <fstream>
#include <gdk/gdk.h>
#include <gtk/gtk.h>

#include "aos/vision/image/reader.h"
#include "aos/vision/image/jpeg_routines.h"
#include "aos/vision/image/image_stream.h"
#include "aos/vision/events/epoll_events.h"
#include "aos/vision/events/tcp_server.h"
#include "aos/vision/blob/stream_view.h"
#include "y2016/vision/blob_filters.h"
// #include "y2016/vision/process_targets.h"

namespace y2016 {
namespace vision {
using namespace aos::vision;

::aos::vision::Vector<2> CreateCenterFromTarget(double lx, double ly, double rx, double ry) {
  return ::aos::vision::Vector<2>((lx + rx) / 2.0, (ly + ry) / 2.0);
}

double TargetWidth(double lx, double ly, double rx, double ry) {
  double dx = lx - rx;
  double dy = ly - ry;
  return ::std::hypot(dx, dy);
}

void SelectTargets(std::vector<std::pair<Vector<2>, Vector<2>>>& left_target,
                   std::vector<std::pair<Vector<2>, Vector<2>>>&right_target,
                   ::aos::vision::Vector<2> *center_left,
                   ::aos::vision::Vector<2> *center_right) {
  // No good targets. Let the caller decide defaults.
  if (right_target.size() == 0 || left_target.size() == 0) {
    return;
  }

  // Only one option, we have to go with it.
  if (right_target.size() == 1 && left_target.size() == 1) {
    *center_left =
        CreateCenterFromTarget(left_target[0].first.x(), left_target[0].first.y(),
                               left_target[0].second.x(), left_target[0].second.y());
    *center_right = CreateCenterFromTarget(
        right_target[0].first.x(), right_target[0].first.y(), right_target[0].second.x(),
        right_target[0].second.y());
    return;
  }

  // Now we have to make a decision.
  double min_angle = -1.0;
  int left_index = 0;
  // First pick the widest target from the left.
  for (size_t i = 0; i < left_target.size(); i++) {
    const double h = left_target[i].first.y() -
                     left_target[i].second.y();
    const double wid1 = TargetWidth(left_target[i].first.x(),
                                    left_target[i].first.y(),
                                    left_target[i].second.x(),
                                    left_target[i].second.y());
    const double angle = h / wid1;
    if (min_angle == -1.0 || ::std::abs(angle) < ::std::abs(min_angle)) {
      min_angle = angle;
      left_index = i;
    }
  }
  // Calculate the angle of the bottom edge for the left.
  double h = left_target[left_index].first.y() -
             left_target[left_index].second.y();

  double good_ang = min_angle;
  double min_ang_err = -1.0;
  int right_index = -1;
  // Now pick the bottom edge angle from the right that lines up best with the left.
  for (size_t j = 0; j < right_target.size(); j++) {
    double wid2 = TargetWidth(right_target[j].first.x(),
                                right_target[j].first.y(),
                                right_target[j].second.x(),
                                right_target[j].second.y());
    h = right_target[j].first.y() -
        right_target[j].second.y();
    double ang = h/ wid2;
    double ang_err = ::std::abs(good_ang - ang);
    if (min_ang_err == -1.0 || min_ang_err > ang_err) {
      min_ang_err = ang_err;
      right_index = j;
    }
  }

  *center_left =
      CreateCenterFromTarget(left_target[left_index].first.x(),
                             left_target[left_index].first.y(),
                             left_target[left_index].second.x(),
                             left_target[left_index].second.y());
  *center_right =
      CreateCenterFromTarget(right_target[right_index].first.x(),
                             right_target[right_index].first.y(),
                             right_target[right_index].second.x(),
                             right_target[right_index].second.y());
}


long GetFileSize(std::string filename) {
  struct stat stat_buf;
  int rc = stat(filename.c_str(), &stat_buf);
  return rc == 0 ? stat_buf.st_size : -1;
}

class OutputFile {
 public:
  OutputFile(const std::string &fname) : ofs(fname, std::ofstream::out) {}

  void Emit(const BlobList &blobl, int64_t timestamp) {
    int tmp_size = CalculateSize(blobl) + sizeof(int32_t) + sizeof(uint64_t);
    tmp_buf.resize(tmp_size, 0);
    {
      char *buf = Int64Codec::Write(&tmp_buf[0], tmp_size);
      buf = Int64Codec::Write(buf, timestamp);
      SerializeBlob(blobl, buf);
    }
    ofs.write(&tmp_buf[0], tmp_size);
    printf("blob_size: %d\n", tmp_size);
  }

  std::vector<char> tmp_buf;

  std::ofstream ofs;
};

class InputFile {
 public:
  InputFile(const std::string &fname)
      : ifs_(fname, std::ifstream::in), len_(GetFileSize(fname)) {
    if (len_ <= 0) {
      printf("File (%s) not found. Size (%d)\n", fname.c_str(), (int)len_);
      fflush(stdout);
    }
    assert(len_ > 0);
    tmp_buf_.resize(len_, 0);
    ifs_.read(&tmp_buf_[0], len_);
    buf_ = &tmp_buf_[0];
  }

  bool ReadNext(BlobList *blob_list, uint64_t *timestamp) {
    if (buf_ - &tmp_buf_[0] >= len_) return false;
    if (prev_ != nullptr) prev_frames_.emplace_back(prev_);
    prev_ = buf_;
    buf_ += sizeof(uint32_t);
    *timestamp = Int64Codec::Read(&buf_);
//    auto* buf_tmp = buf_;
    buf_ = ParseBlobList(blob_list, buf_);
//    fprintf(stderr, "read frame: %lu, buf_size: %lu\n", *timestamp, buf_ - buf_tmp);
    return true;
  }

  bool ReadPrev(BlobList *blob_list, uint64_t *timestamp) {
    if (prev_frames_.empty()) return false;
    buf_ = prev_frames_.back();
    prev_frames_.pop_back();
    buf_ += sizeof(uint32_t);
    *timestamp = Int64Codec::Read(&buf_);
    buf_ = ParseBlobList(blob_list, buf_);
    prev_ = nullptr;
    return true;
  }

 private:
  std::vector<const char *> prev_frames_;
  const char *buf_;
  const char *prev_ = nullptr;
  std::ifstream ifs_;

  long len_;
  std::vector<char> tmp_buf_;
};

class BlobStreamFrame {
 public:
  BlobList blob_list;
  uint64_t timestamp;
  void ReadNext(InputFile *fin) {
    blob_list.clear();
    if (!fin->ReadNext(&blob_list, &timestamp)) {
      exit(0);
      return;
    }
  }
  bool ReadPrev(InputFile *fin) {
    blob_list.clear();
    return fin->ReadPrev(&blob_list, &timestamp);
  }
};

const char *kHudText =
    "commands:\n"
    " SPACE - pause\n"
    " c - continue to next target\n"
    " s - single step while paused\n"
    " k - pause on next target frame\n"
    " u - change window scaling\n"
    " a - single step backward\n"
    " q - quit\n"
    " h - help\n";

class NetworkForwardingImageStream : public aos::events::EpollWait {
 public:
  NetworkForwardingImageStream(ImageFormat fmt, int debug_level,
                               const std::string &fname1,
                               const std::string &fname2)
      : fmt_(fmt),
        ifs1_(fname1),
        ifs2_(fname2),
        blob_filt_(fmt, 40, 750, 250000),
        finder_(0.25, 35) {
    text_overlay_.draw_fn =
        [this](RenderInterface *render, double /*width*/, double /*height*/) {
      render->SetSourceRGB(1.0, 1.0, 1.0);
      if (hud_text) render->Text(20, 20, 0, 0, kHudText);
    };

    overlays_.push_back(&overlay_);
    overlays_.push_back(&text_overlay_);
    view_.view()->SetOverlays(&overlays_);

    if (debug_level > 0) {
      finder_.EnableOverlay(&overlay_);
    }
    if (debug_level > 1) {
      blob_filt_.EnableOverlay(&overlay_);
    }

    frame1.ReadNext(&ifs1_);
    frame2.ReadNext(&ifs2_);

    std::pair<int, int> skip =
        TickFrame(std::max(frame1.timestamp, frame2.timestamp));
    printf("Initialzation skipped (%d, %d)\n", skip.first, skip.second);

    ms_event_delta_ = 20;
    play_forward = true;
    paused = false;
    single_step = false;
    pause_on_next_target = true;
    continue_to_next_target = false;
    view_.view()->SetScale(scale_factor);
    view_.view()->key_press_event = [this](uint32_t keyval) {
      play_forward = true;
      switch (keyval) {
        case GDK_KEY_space:
          paused = !paused;
          pause_on_next_target = false;
          continue_to_next_target = false;
          break;
        case GDK_KEY_c:
          pause_on_next_target = true;
          continue_to_next_target = true;
          paused = false;
          break;
        case GDK_KEY_s:
          single_step = true;
          continue_to_next_target = false;
          paused = true;
          break;
        case GDK_KEY_k:
          pause_on_next_target = true;
          continue_to_next_target = false;
          paused = false;
          break;
        case GDK_KEY_u:
          if (scale_factor == 1.0) {
            scale_factor = 0.75;
            view_.view()->SetScale(0.75);
          } else {
            scale_factor = 1.0;
            view_.view()->SetScale(1.0);
            view_.view()->MoveTo(150, -220);
          }
          break;
        case GDK_KEY_a:
          play_forward = false;
          single_step = true;
          paused = true;
          break;
        case GDK_KEY_q:
          exit(0);
        case GDK_KEY_h:
          hud_text = !hud_text;
          break;
        default:
          printf("pressed: %s\n", gdk_keyval_name(keyval));
      }
    };
  }

  double scale_factor = 1.0;
  bool hud_text = true;
  bool play_forward;
  bool paused;
  bool single_step;
  bool pause_on_next_target;
  bool continue_to_next_target;

  std::string distance_text;

  std::pair<int, int> TickFrame(uint64_t time) {
    timestamp_ += time;
    return TickToFrame(timestamp_);
  }

  std::pair<int, int> TickBackFrame(uint64_t time) {
    timestamp_ -= time;
    return TickBackToFrame(timestamp_);
  }

  std::pair<int, int> TickToFrame(uint64_t timestamp) {
    std::pair<int, int> skip(0, 0);
    while (frame1.timestamp < timestamp) {
      frame1.ReadNext(&ifs1_);
      skip.first++;
    }
    while (frame2.timestamp < timestamp) {
      frame2.ReadNext(&ifs2_);
      skip.second++;
    }
    return skip;
  }

  std::pair<int, int> TickBackToFrame(uint64_t timestamp) {
    std::pair<int, int> skip(0, 0);
    while (frame1.timestamp >= timestamp) {
      if (!frame1.ReadPrev(&ifs1_)) break;
      skip.first++;
    }
    while (frame2.timestamp >= timestamp) {
      if (!frame2.ReadPrev(&ifs2_)) break;
      skip.second++;
    }
    frame1.ReadPrev(&ifs1_);
    frame2.ReadPrev(&ifs2_);
    return skip;
  }
  BlobStreamFrame frame1;
  BlobStreamFrame frame2;
  uint64_t timestamp_ = 0;

  Vector<2> GetCenter(const BlobList &blob_list) {
    std::vector<std::pair<Vector<2>, Vector<2>>> corners =
        finder_.Find(blob_filt_.FilterBlobs(blob_list));

    if (corners.size() == 1) {
      Vector<2> center = (corners[0].first + corners[0].second) * (0.5);
      return center;
    }
    return {0, 0};
  }

  void DrawSuperSpeed() {
    PixelRef color = {0, 255, 255};
    // S
    overlay_.AddLine(Vector<2>(200, 100), Vector<2>(100, 100), color);
    overlay_.AddLine(Vector<2>(100, 100), Vector<2>(100, 300), color);
    overlay_.AddLine(Vector<2>(100, 300), Vector<2>(200, 300), color);
    overlay_.AddLine(Vector<2>(200, 300), Vector<2>(200, 500), color);
    overlay_.AddLine(Vector<2>(200, 500), Vector<2>(100, 500), color);
    // U
    overlay_.AddLine(Vector<2>(250, 100), Vector<2>(250, 500), color);
    overlay_.AddLine(Vector<2>(250, 500), Vector<2>(350, 500), color);
    overlay_.AddLine(Vector<2>(350, 500), Vector<2>(350, 100), color);
    // P
    overlay_.AddLine(Vector<2>(400, 100), Vector<2>(400, 500), color);
    overlay_.AddLine(Vector<2>(400, 100), Vector<2>(500, 100), color);
    overlay_.AddLine(Vector<2>(500, 100), Vector<2>(500, 300), color);
    overlay_.AddLine(Vector<2>(500, 300), Vector<2>(400, 300), color);
    // E
    overlay_.AddLine(Vector<2>(550, 100), Vector<2>(550, 500), color);
    overlay_.AddLine(Vector<2>(550, 100), Vector<2>(650, 100), color);
    overlay_.AddLine(Vector<2>(550, 300), Vector<2>(650, 300), color);
    overlay_.AddLine(Vector<2>(550, 500), Vector<2>(650, 500), color);
    // R
    overlay_.AddLine(Vector<2>(700, 100), Vector<2>(700, 500), color);
    overlay_.AddLine(Vector<2>(700, 100), Vector<2>(800, 100), color);
    overlay_.AddLine(Vector<2>(800, 100), Vector<2>(800, 300), color);
    overlay_.AddLine(Vector<2>(800, 300), Vector<2>(700, 300), color);
    overlay_.AddLine(Vector<2>(700, 350), Vector<2>(800, 500), color);
    // S
    overlay_.AddLine(Vector<2>(200, 550), Vector<2>(100, 550), color);
    overlay_.AddLine(Vector<2>(100, 550), Vector<2>(100, 750), color);
    overlay_.AddLine(Vector<2>(100, 750), Vector<2>(200, 750), color);
    overlay_.AddLine(Vector<2>(200, 750), Vector<2>(200, 950), color);
    overlay_.AddLine(Vector<2>(200, 950), Vector<2>(100, 950), color);
    // P
    overlay_.AddLine(Vector<2>(250, 550), Vector<2>(250, 950), color);
    overlay_.AddLine(Vector<2>(250, 550), Vector<2>(350, 550), color);
    overlay_.AddLine(Vector<2>(350, 550), Vector<2>(350, 750), color);
    overlay_.AddLine(Vector<2>(350, 750), Vector<2>(250, 750), color);
    // E
    overlay_.AddLine(Vector<2>(400, 550), Vector<2>(400, 950), color);
    overlay_.AddLine(Vector<2>(400, 550), Vector<2>(500, 550), color);
    overlay_.AddLine(Vector<2>(400, 750), Vector<2>(500, 750), color);
    overlay_.AddLine(Vector<2>(400, 950), Vector<2>(500, 950), color);
    // E
    overlay_.AddLine(Vector<2>(550, 550), Vector<2>(550, 950), color);
    overlay_.AddLine(Vector<2>(550, 550), Vector<2>(650, 550), color);
    overlay_.AddLine(Vector<2>(550, 750), Vector<2>(650, 750), color);
    overlay_.AddLine(Vector<2>(550, 950), Vector<2>(650, 950), color);
    // D
    overlay_.AddLine(Vector<2>(700, 550), Vector<2>(700, 950), color);
    overlay_.AddLine(Vector<2>(700, 550), Vector<2>(800, 575), color);
    overlay_.AddLine(Vector<2>(800, 575), Vector<2>(800, 925), color);
    overlay_.AddLine(Vector<2>(800, 925), Vector<2>(700, 950), color);
  }

  void UpdateNewTime(int new_delta) {
    if (new_delta != ms_event_delta_) {
      ms_event_delta_ = new_delta;
      SetTime(::std::chrono::milliseconds(ms_event_delta_) + aos::monotonic_clock::now());
    }
  }

  void Done() override {
    SetTime(::std::chrono::milliseconds(ms_event_delta_) + aos::monotonic_clock::now());
    if (paused && !single_step) return;
    single_step = false;
    frame_count_++;

    while (true) {
      overlay_.Reset();
      view_.SetFormatAndClear(fmt_);
      std::pair<int, int> skipped(1, 1);
      // how far we will step to look for the next target
      int nano_step = 300 * 1e6;
      if (play_forward && seeking_target_) {
        skipped = TickFrame(nano_step);
      } else if (seeking_target_) {
        skipped = TickBackFrame(nano_step);
      } else if (play_forward) {
        frame1.ReadNext(&ifs1_);
        frame2.ReadNext(&ifs2_);
      } else {
        frame1.ReadPrev(&ifs1_);
        frame2.ReadPrev(&ifs2_);
      }
      // printf("skipped (%d, %d)\n", skipped.first, skipped.second);

      std::vector<std::pair<Vector<2>, Vector<2>>> corner1 =
          finder_.Find(blob_filt_.FilterBlobs(frame1.blob_list));
      std::vector<std::pair<Vector<2>, Vector<2>>> corner2 =
          finder_.Find(blob_filt_.FilterBlobs(frame2.blob_list));

      Vector<2> cent1;
      Vector<2> cent2;
      SelectTargets(corner1, corner2, &cent1, &cent2);

      /*
      int target_count_;
      if (cent1 == Vector<2>(0, 0) && cent2 == Vector<2>(0, 0)) {
        missed_count_ += std::min(skipped.first, skipped.second);
        if (missed_count_ > 15) {
          seeking_target_ = true;
          DrawSuperSpeed();
          SetTime(aos::vision::MsTime(1));
          if (line_break_) {
            printf("_-_-_-%d_-_-_-_\n", target_count_);
            target_count_++;
            line_break_ = false;
          }
          if (continue_to_next_target) {
            continue_to_next_target = false;
          }
          continue;
        }
      } else {
        missed_count_ = 0;
      }
      */

      if (seeking_target_) {
        if (play_forward) {
          // Go back to the last time we didn't see a target and play from there.
          TickBackFrame(nano_step);
          seeking_target_ = false;
        } else if (seeking_target_) {
          TickFrame(nano_step);
          seeking_target_ = false;
        }
        continue;
      }

      // comment out to turn off full blob drawing
      view_.DrawBlobList(frame1.blob_list, {0, 0, 255});
      view_.DrawSecondBlobList(frame2.blob_list, {0, 255, 0}, {0, 255, 255});

      DrawCross(overlay_, Vector<2>(fmt_.w / 2.0, fmt_.h / 2.0), {255, 0, 0});

      double timeFromEpoch =
          1e-9 * ((double)frame1.timestamp + (double)frame2.timestamp) / 2;
      printf("timestamp: %g skew: %g\n", timeFromEpoch,
             1e-9 * ((double)frame1.timestamp - (double)frame2.timestamp));
      /*
      if (cent1 == Vector<2>(0, 0) && cent2 == Vector<2>(0, 0)) {
      } else {
        DrawCross(overlay_, cent1, {255, 255, 255});
        DrawCross(overlay_, cent2, {255, 255, 255});
        double x = (cent1.x() + cent2.x()) / 2.0;
        DrawCross(overlay_, Vector<2>(x, fmt_.h / 2.0), {255, 255, 255});
        SetTime(aos::vision::MsTime(100));
        if (pause_on_next_target && !continue_to_next_target) {
          paused = true;
          pause_on_next_target = false;
        }
        line_break_ = true;
        missed_count_ = 0;
      }
      fflush(stdout);
      */
      view_.view()->Redraw();
      break;
    }
  }

  void DrawCross(PixelLinesOverlay &overlay, Vector<2> center, PixelRef color) {
    overlay.AddLine(Vector<2>(center.x() - 25, center.y()),
                     Vector<2>(center.x() + 25, center.y()), color);
    overlay.AddLine(Vector<2>(center.x(), center.y() - 25),
                     Vector<2>(center.x(), center.y() + 25), color);
  }

  void AddTo(aos::events::EpollLoop *loop) {
    Done();
    loop->AddWait(this);
  }

  std::unique_ptr<PixelRef[]> outbuf;
  ImagePtr ptr;

  BlobStreamViewer view_;

 private:
  int ms_event_delta_ = 200;
 public:
  // basic image size
  ImageFormat fmt_;

  // where we darw for debugging
  PixelLinesOverlay overlay_;

  // Where we draw text on the screen.
  LambdaOverlay text_overlay_;
  // container for viewer
  std::vector<OverlayBase *> overlays_;

  InputFile ifs1_;
  InputFile ifs2_;

  // our blob processing object
  HistogramBlobFilter blob_filt_;

  // corner finder to align aiming
  CornerFinder finder_;

  // indicates we have lost a target
  bool line_break_ = false;

  // indicates we are looking for the next target
  bool seeking_target_ = false;

  int frame_count_ = 0;

  // count how many frames we miss in a row.
  int missed_count_ = 16;
};
}
}  // namespace y2016::vision

int main(int argc, char *argv[]) {
  using namespace y2016::vision;
  aos::events::EpollLoop loop;
  gtk_init(&argc, &argv);

  if (argc != 3) {
    printf("Wrong number of arguments. Got (%d) expected 3.\n", argc);
    printf(
        " arguments are debug level as {0, 1, 2} and then filename without the "
        "{_0.dat,_1.dat} suffixes\n");
  }

  int dbg = std::stoi(argv[1]);

  std::string file(argv[2]);
  aos::vision::ImageFormat fmt = {640 * 2, 480 * 2};

  printf("file (%s) dbg_lvl (%d)\n", file.c_str(), dbg);

  std::string fname_path = file;
  NetworkForwardingImageStream strm1(
      fmt, dbg, fname_path + "_0.dat", fname_path + "_1.dat");
  fprintf(stderr, "staring main\n");
  strm1.AddTo(&loop);

  fprintf(stderr, "staring main\n");
  loop.RunWithGtkMain();
}
