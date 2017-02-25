#include "aos/vision/debug/debug_framework.h"

#include <gtk/gtk.h>

#include "aos/common/logging/implementations.h"
#include "aos/common/logging/logging.h"
#include "aos/vision/blob/find_blob.h"
#include "aos/vision/blob/stream_view.h"
#include "aos/vision/debug/debug_viewer.h"
#include "aos/vision/events/epoll_events.h"
#include "aos/vision/image/jpeg_routines.h"

namespace aos {
namespace vision {

bool DecodeJpeg(aos::vision::DataRef data,
                aos::vision::BlobStreamViewer *view) {
  auto fmt = aos::vision::GetFmt(data);
  auto value = view->img();
  if (!value.fmt().Equals(fmt)) {
    view->SetFormatAndClear(fmt);
  }
  return aos::vision::ProcessJpeg(data, view->img().data());
}

class DebugFramework : public DebugFrameworkInterface {
 public:
  explicit DebugFramework(FilterHarness *filter) : filter_(filter) {
    view_.key_press_event = [this](uint32_t keyval) {
      for (const auto &event : key_press_events()) {
        event(keyval);
      }
    };
    filter->InstallViewer(&view_);
  }

  // This the first stage in the pipeline that takes
  void NewJpeg(DataRef data) override {
    DecodeJpeg(data, &view_);

    auto fmt = view_.img().fmt();
    HandleBlobs(FindBlobs(filter_->Threshold(view_.img())), fmt);
  }

  void NewBlobList(BlobList blob_list, ImageFormat fmt) override {
    view_.SetFormatAndClear(fmt);

    HandleBlobs(std::move(blob_list), fmt);
  }

  void HandleBlobs(BlobList blob_list, ImageFormat fmt) {
    filter_->HandleBlobs(std::move(blob_list), fmt);
    view_.Redraw();
  }

  aos::events::EpollLoop *Loop() override { return &loop_; }

 private:
  FilterHarness *filter_;
  BlobStreamViewer view_;

  aos::events::EpollLoop loop_;
};

std::unique_ptr<ImageSource> MakeImageSource(
    const std::string &image_source_string,
    DebugFrameworkInterface *interface) {
  (void)interface;
  // Each of the image_source strings is of the form format_type:format_spec
  auto it = image_source_string.find(':');
  if (it == std::string::npos) {
    fprintf(stderr, "invalid ImageSource: %s.\n", image_source_string.c_str());
    exit(-1);
  }
  auto image_source_type = image_source_string.substr(0, it);
  // Get std::function<std::unique_ptr<ImageSource>()> from the registration
  // factory.
  const auto &factory = ImageSourceGlobalFactory::Get(image_source_type);
  if (!factory) {
    fprintf(stderr, "invalid ImageSource: %s.\n", image_source_string.c_str());
    exit(-1);
  }
  auto result = factory();
  // Construct the image source.
  result->Init(image_source_string.substr(it + 1), interface);
  return result;
}

const char *kHelpMessage = R"(

image_source is parsed out and selects where to get the images
from. Each source type has a different configuration format string listed
below. The colon separates the source specifier and the source config
parameter. A single command line argument help will print this message.
)";

void DebugFrameworkMain(int argc, char **argv, FilterHarness *filter) {
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));

  gtk_init(&argc, &argv);

  // Use fprintf because it is only supposed to be used interactively.
  // This uses a registration system to pick out the individual file type
  // registered by REGISTER_IMAGE_SOURCE.
  // see jpeg_list-source.cc for ane sample of this.
  if (argc < 2 || argv[1] == std::string("help")) {
    fprintf(stderr, "Usage %s image_source:format_spec\n", argv[0]);
    fprintf(stderr, "%s", kHelpMessage);
    // Iterate through all registered entities in ImageSourceGlobalFactory
    // and print out their individual help messages.
    for (const auto &type : ImageSourceGlobalFactory::GetAll()) {
      fprintf(stderr, "  %s:\n", type.first.c_str());
      fprintf(stderr, "%s", type.second()->GetHelpMessage());
    }
    exit(-1);
  }

  DebugFramework replay(filter);

  std::unique_ptr<ImageSource> image_source = MakeImageSource(argv[1], &replay);

  replay.Loop()->RunWithGtkMain();
}

}  // namespace vision
}  // namespace aos
