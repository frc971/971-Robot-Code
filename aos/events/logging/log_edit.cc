#include <iostream>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/file.h"

ABSL_FLAG(std::string, logfile, "/tmp/logfile.bfbs",
          "Name of the logfile to read from.");
ABSL_FLAG(bool, replace, false,
          "If true, replace the header on the log file with the JSON header.");
ABSL_FLAG(
    std::string, header, "",
    "If provided, this is the path to the JSON with the log file header.  If "
    "not provided, _header.json will be appended to --logfile.");

ABSL_FLAG(int32_t, max_message_size, 128 * 1024 * 1024,
          "Max size of a message to be written.  This sets the buffers inside "
          "the encoders.");
ABSL_FLAG(bool, direct, false,
          "If true, write using O_DIRECT and write 512 byte aligned blocks "
          "whenever possible.");

int main(int argc, char **argv) {
  absl::SetProgramUsageMessage(R"(This tool lets us manipulate log files.)");
  aos::InitGoogle(&argc, &argv);

  std::string header_json_path =
      absl::GetFlag(FLAGS_header).empty()
          ? (absl::GetFlag(FLAGS_logfile) + "_header.json")
          : absl::GetFlag(FLAGS_header);

  if (absl::GetFlag(FLAGS_replace)) {
    const ::std::string header_json =
        aos::util::ReadFileToStringOrDie(header_json_path);
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    flatbuffers::Offset<aos::logger::LogFileHeader> header_offset =
        aos::JsonToFlatbuffer<aos::logger::LogFileHeader>(header_json, &fbb);

    fbb.FinishSizePrefixed(header_offset);
    aos::SizePrefixedFlatbufferDetachedBuffer<aos::logger::LogFileHeader>
        header(fbb.Release());

    const std::string orig_path = absl::GetFlag(FLAGS_logfile) + ".orig";
    PCHECK(rename(absl::GetFlag(FLAGS_logfile).c_str(), orig_path.c_str()) ==
           0);

    aos::logger::SpanReader span_reader(orig_path);
    CHECK(!span_reader.ReadMessage().empty()) << ": Empty header, aborting";

    aos::logger::FileBackend file_backend("/", absl::GetFlag(FLAGS_direct));
    aos::logger::DetachedBufferWriter buffer_writer(
        file_backend.RequestFile(absl::GetFlag(FLAGS_logfile)),
        std::make_unique<aos::logger::DummyEncoder>(
            absl::GetFlag(FLAGS_max_message_size)));
    {
      aos::logger::DataEncoder::SpanCopier copier(header.span());
      buffer_writer.CopyMessage(&copier, aos::monotonic_clock::min_time);
    }

    while (true) {
      absl::Span<const uint8_t> msg_data = span_reader.ReadMessage();
      if (msg_data.empty()) {
        break;
      }

      {
        aos::logger::DataEncoder::SpanCopier copier(msg_data);
        buffer_writer.CopyMessage(&copier, aos::monotonic_clock::min_time);
      }
    }
  } else {
    aos::logger::MessageReader reader(absl::GetFlag(FLAGS_logfile));
    aos::util::WriteStringToFileOrDie(
        header_json_path,
        aos::FlatbufferToJson(reader.log_file_header(), {.multi_line = true}));
  }

  return 0;
}
