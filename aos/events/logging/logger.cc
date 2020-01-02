#include "aos/events/logging/logger.h"

#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "flatbuffers/flatbuffers.h"

DEFINE_int32(flush_size, 1000000,
             "Number of outstanding bytes to allow before flushing to disk.");
DEFINE_bool(skip_missing_forwarding_entries, false,
            "If true, drop any forwarding entries with missing data.  If "
            "false, CHECK.");

namespace aos {
namespace logger {

namespace chrono = std::chrono;

DetachedBufferWriter::DetachedBufferWriter(absl::string_view filename)
    : fd_(open(std::string(filename).c_str(),
               O_RDWR | O_CLOEXEC | O_CREAT | O_EXCL, 0774)) {
  PCHECK(fd_ != -1) << ": Failed to open " << filename;
}

DetachedBufferWriter::~DetachedBufferWriter() {
  Flush();
  PLOG_IF(ERROR, close(fd_) == -1) << " Failed to close logfile";
}

void DetachedBufferWriter::QueueSizedFlatbuffer(
    flatbuffers::FlatBufferBuilder *fbb) {
  QueueSizedFlatbuffer(fbb->Release());
}

void DetachedBufferWriter::QueueSizedFlatbuffer(
    flatbuffers::DetachedBuffer &&buffer) {
  queued_size_ += buffer.size();
  queue_.emplace_back(std::move(buffer));

  // Flush if we are at the max number of iovs per writev, or have written
  // enough data.  Otherwise writev will fail with an invalid argument.
  if (queued_size_ > static_cast<size_t>(FLAGS_flush_size) ||
      queue_.size() == IOV_MAX) {
    Flush();
  }
}

void DetachedBufferWriter::Flush() {
  if (queue_.size() == 0u) {
    return;
  }
  iovec_.clear();
  iovec_.reserve(queue_.size());
  size_t counted_size = 0;
  for (size_t i = 0; i < queue_.size(); ++i) {
    struct iovec n;
    n.iov_base = queue_[i].data();
    n.iov_len = queue_[i].size();
    counted_size += n.iov_len;
    iovec_.emplace_back(std::move(n));
  }
  CHECK_EQ(counted_size, queued_size_);
  const ssize_t written = writev(fd_, iovec_.data(), iovec_.size());

  PCHECK(written == static_cast<ssize_t>(queued_size_))
      << ": Wrote " << written << " expected " << queued_size_;

  queued_size_ = 0;
  queue_.clear();
  // TODO(austin): Handle partial writes in some way other than crashing...
}

Logger::Logger(DetachedBufferWriter *writer, EventLoop *event_loop,
               std::chrono::milliseconds polling_period)
    : event_loop_(event_loop),
      writer_(writer),
      timer_handler_(event_loop_->AddTimer([this]() { DoLogData(); })),
      polling_period_(polling_period) {
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    FetcherStruct fs;
    const bool is_readable =
        configuration::ChannelIsReadableOnNode(channel, event_loop_->node());
    const bool log_message = configuration::ChannelMessageIsLoggedOnNode(
                                 channel, event_loop_->node()) &&
                             is_readable;

    const bool log_delivery_times =
        (event_loop_->node() == nullptr)
            ? false
            : configuration::ConnectionDeliveryTimeIsLoggedOnNode(
                  channel, event_loop_->node(), event_loop_->node());

    if (log_message || log_delivery_times) {
      fs.fetcher = event_loop->MakeRawFetcher(channel);
      VLOG(1) << "Logging channel "
              << configuration::CleanedChannelToString(channel);

      if (log_delivery_times) {
        if (log_message) {
          VLOG(1) << "  Logging message and delivery times";
          fs.log_type = LogType::kLogMessageAndDeliveryTime;
        } else {
          VLOG(1) << "  Logging delivery times only";
          fs.log_type = LogType::kLogDeliveryTimeOnly;
        }
      } else {
        // We don't have a particularly great use case right now for logging a
        // forwarded message, but either not logging the delivery times, or
        // logging them on another node.  Fail rather than produce bad results.
        CHECK(configuration::ChannelIsSendableOnNode(channel,
                                                     event_loop_->node()))
            << ": Logger only knows how to log remote messages with "
               "forwarding timestamps.";
        VLOG(1) << "  Logging message only";
        fs.log_type = LogType::kLogMessage;
      }
    }

    fs.written = false;
    fetchers_.emplace_back(std::move(fs));
  }

  // When things start, we want to log the header, then the most recent messages
  // available on each fetcher to capture the previous state, then start
  // polling.
  event_loop_->OnRun([this, polling_period]() {
    // Grab data from each channel right before we declare the log file started
    // so we can capture the latest message on each channel.  This lets us have
    // non periodic messages with configuration that now get logged.
    for (FetcherStruct &f : fetchers_) {
      if (f.fetcher.get() != nullptr) {
        f.written = !f.fetcher->Fetch();
      }
    }

    // We need to pick a point in time to declare the log file "started".  This
    // starts here.  It needs to be after everything is fetched so that the
    // fetchers are all pointed at the most recent message before the start
    // time.
    const monotonic_clock::time_point monotonic_now =
        event_loop_->monotonic_now();
    const realtime_clock::time_point realtime_now = event_loop_->realtime_now();
    last_synchronized_time_ = monotonic_now;

    {
      // Now write the header with this timestamp in it.
      flatbuffers::FlatBufferBuilder fbb;
      fbb.ForceDefaults(1);

      flatbuffers::Offset<aos::Configuration> configuration_offset =
          CopyFlatBuffer(event_loop_->configuration(), &fbb);

      flatbuffers::Offset<flatbuffers::String> string_offset =
          fbb.CreateString(network::GetHostname());

      flatbuffers::Offset<Node> node_offset;
      if (event_loop_->node() != nullptr) {
        node_offset = CopyFlatBuffer(event_loop_->node(), &fbb);
      }
      LOG(INFO) << "Logging node as " << FlatbufferToJson(event_loop_->node());

      aos::logger::LogFileHeader::Builder log_file_header_builder(fbb);

      log_file_header_builder.add_name(string_offset);

      // Only add the node if we are running in a multinode configuration.
      if (event_loop_->node() != nullptr) {
        log_file_header_builder.add_node(node_offset);
      }

      log_file_header_builder.add_configuration(configuration_offset);
      // The worst case theoretical out of order is the polling period times 2.
      // One message could get logged right after the boundary, but be for right
      // before the next boundary.  And the reverse could happen for another
      // message.  Report back 3x to be extra safe, and because the cost isn't
      // huge on the read side.
      log_file_header_builder.add_max_out_of_order_duration(
          std::chrono::duration_cast<std::chrono::nanoseconds>(3 *
                                                               polling_period)
              .count());

      log_file_header_builder.add_monotonic_start_time(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              monotonic_now.time_since_epoch())
              .count());
      log_file_header_builder.add_realtime_start_time(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              realtime_now.time_since_epoch())
              .count());

      fbb.FinishSizePrefixed(log_file_header_builder.Finish());
      writer_->QueueSizedFlatbuffer(&fbb);
    }

    timer_handler_->Setup(event_loop_->monotonic_now() + polling_period,
                          polling_period);
  });
}

flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, LogType log_type) {
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset;

  switch(log_type) {
    case LogType::kLogMessage:
    case LogType::kLogMessageAndDeliveryTime:
      data_offset =
          fbb->CreateVector(static_cast<uint8_t *>(context.data), context.size);
      break;

    case LogType::kLogDeliveryTimeOnly:
      break;
  }

  MessageHeader::Builder message_header_builder(*fbb);
  message_header_builder.add_channel_index(channel_index);
  message_header_builder.add_queue_index(context.queue_index);
  message_header_builder.add_monotonic_sent_time(
      context.monotonic_event_time.time_since_epoch().count());
  message_header_builder.add_realtime_sent_time(
      context.realtime_event_time.time_since_epoch().count());

  switch (log_type) {
    case LogType::kLogMessage:
      message_header_builder.add_data(data_offset);
      break;

    case LogType::kLogMessageAndDeliveryTime:
      message_header_builder.add_data(data_offset);
      [[fallthrough]];

    case LogType::kLogDeliveryTimeOnly:
      message_header_builder.add_monotonic_remote_time(
          context.monotonic_remote_time.time_since_epoch().count());
      message_header_builder.add_realtime_remote_time(
          context.realtime_remote_time.time_since_epoch().count());
      message_header_builder.add_remote_queue_index(context.remote_queue_index);
      break;
  }

  return message_header_builder.Finish();
}

void Logger::DoLogData() {
  // We want to guarentee that messages aren't out of order by more than
  // max_out_of_order_duration.  To do this, we need sync points.  Every write
  // cycle should be a sync point.
  const monotonic_clock::time_point monotonic_now = monotonic_clock::now();

  do {
    // Move the sync point up by at most polling_period.  This forces one sync
    // per iteration, even if it is small.
    last_synchronized_time_ =
        std::min(last_synchronized_time_ + polling_period_, monotonic_now);
    size_t channel_index = 0;
    // Write each channel to disk, one at a time.
    for (FetcherStruct &f : fetchers_) {
      // Skip any channels which we aren't supposed to log.
      if (f.fetcher.get() != nullptr) {
        while (true) {
          if (f.written) {
            if (!f.fetcher->FetchNext()) {
              VLOG(2) << "No new data on "
                      << configuration::CleanedChannelToString(
                             f.fetcher->channel());
              break;
            } else {
              f.written = false;
            }
          }

          CHECK(!f.written);

          // TODO(james): Write tests to exercise this logic.
          if (f.fetcher->context().monotonic_event_time <
              last_synchronized_time_) {
            // Write!
            flatbuffers::FlatBufferBuilder fbb(f.fetcher->context().size +
                                               max_header_size_);
            fbb.ForceDefaults(1);

            fbb.FinishSizePrefixed(PackMessage(&fbb, f.fetcher->context(),
                                               channel_index, f.log_type));

            VLOG(2) << "Writing data for channel "
                    << configuration::CleanedChannelToString(
                           f.fetcher->channel());

            max_header_size_ = std::max(
                max_header_size_, fbb.GetSize() - f.fetcher->context().size);
            writer_->QueueSizedFlatbuffer(&fbb);

            f.written = true;
          } else {
            break;
          }
        }
      }

      ++channel_index;
    }

    CHECK_EQ(channel_index, fetchers_.size());

    // If we missed cycles, we could be pretty far behind.  Spin until we are
    // caught up.
  } while (last_synchronized_time_ + polling_period_ < monotonic_now);

  writer_->Flush();
}

LogReader::LogReader(absl::string_view filename)
    : fd_(open(std::string(filename).c_str(), O_RDONLY | O_CLOEXEC)) {
  PCHECK(fd_ != -1) << ": Failed to open " << filename;

  // Make sure we have enough to read the size.
  absl::Span<const uint8_t> config_data = ReadMessage();

  // Make sure something was read.
  CHECK(config_data != absl::Span<const uint8_t>());

  // And copy the config so we have it forever.
  configuration_ = std::vector<uint8_t>(config_data.begin(), config_data.end());

  max_out_of_order_duration_ = std::chrono::nanoseconds(
      flatbuffers::GetSizePrefixedRoot<LogFileHeader>(configuration_.data())
          ->max_out_of_order_duration());

  channels_.resize(configuration()->channels()->size());

  QueueMessages();
}

LogReader::~LogReader() {
  CHECK(!event_loop_unique_ptr_) << "Did you remember to call Deregister?";
}

bool LogReader::ReadBlock() {
  if (end_of_file_) {
    return false;
  }

  // Appends 256k.  This is enough that the read call is efficient.  We don't
  // want to spend too much time reading small chunks because the syscalls for
  // that will be expensive.
  constexpr size_t kReadSize = 256 * 1024;

  // Strip off any unused data at the front.
  if (consumed_data_ != 0) {
    data_.erase(data_.begin(), data_.begin() + consumed_data_);
    consumed_data_ = 0;
  }

  const size_t starting_size = data_.size();

  // This should automatically grow the backing store.  It won't shrink if we
  // get a small chunk later.  This reduces allocations when we want to append
  // more data.
  data_.resize(data_.size() + kReadSize);

  ssize_t count = read(fd_, &data_[starting_size], kReadSize);
  data_.resize(starting_size + std::max(count, static_cast<ssize_t>(0)));
  if (count == 0) {
    end_of_file_ = true;
    return false;
  }
  PCHECK(count > 0);

  return true;
}

bool LogReader::MessageAvailable() {
  // Are we big enough to read the size?
  if (data_.size() - consumed_data_ < sizeof(flatbuffers::uoffset_t)) {
    return false;
  }

  // Then, are we big enough to read the full message?
  const size_t data_size =
      flatbuffers::GetPrefixedSize(data_.data() + consumed_data_) +
      sizeof(flatbuffers::uoffset_t);
  if (data_.size() < consumed_data_ + data_size) {
    return false;
  }

  return true;
}

absl::Span<const uint8_t> LogReader::ReadMessage() {
  // Make sure we have enough for the size.
  if (data_.size() - consumed_data_ < sizeof(flatbuffers::uoffset_t)) {
    if (!ReadBlock()) {
      return absl::Span<const uint8_t>();
    }
  }

  // Now make sure we have enough for the message.
  const size_t data_size =
      flatbuffers::GetPrefixedSize(data_.data() + consumed_data_) +
      sizeof(flatbuffers::uoffset_t);
  while (data_.size() < consumed_data_ + data_size) {
    if (!ReadBlock()) {
      return absl::Span<const uint8_t>();
    }
  }

  // And return it, consuming the data.
  const uint8_t *data_ptr = data_.data() + consumed_data_;

  consumed_data_ += data_size;

  return absl::Span<const uint8_t>(data_ptr, data_size);
}

void LogReader::QueueMessages() {
  while (true) {
    // Don't queue if we have enough data already.
    // When a log file starts, there should be a message from each channel.
    // Those messages might be very old. Make sure to read a chunk past the
    // starting time.
    if (channel_heap_.size() > 0 &&
        newest_timestamp_ >
            std::max(oldest_message().first, monotonic_start_time()) +
                max_out_of_order_duration_) {
      break;
    }

    absl::Span<const uint8_t> msg_data = ReadMessage();
    if (msg_data == absl::Span<const uint8_t>()) {
      break;
    }

    FlatbufferVector<MessageHeader> msg(std::vector<uint8_t>(
        msg_data.begin() + sizeof(flatbuffers::uoffset_t), msg_data.end()));

    EmplaceDataBack(std::move(msg));
  }

  queue_data_time_ = newest_timestamp_ - max_out_of_order_duration_;
}

const Configuration *LogReader::configuration() const {
  return flatbuffers::GetSizePrefixedRoot<LogFileHeader>(configuration_.data())
      ->configuration();
}

const Node *LogReader::node() const {
  if (configuration()->has_nodes()) {
    CHECK(flatbuffers::GetSizePrefixedRoot<LogFileHeader>(configuration_.data())
              ->has_node());
    CHECK(flatbuffers::GetSizePrefixedRoot<LogFileHeader>(configuration_.data())
              ->node()
              ->has_name());
    return configuration::GetNode(
        configuration(),
        flatbuffers::GetSizePrefixedRoot<LogFileHeader>(configuration_.data())
            ->node()
            ->name()
            ->string_view());
  } else {
    CHECK(
        !flatbuffers::GetSizePrefixedRoot<LogFileHeader>(configuration_.data())
             ->has_node());
    return nullptr;
  }
}

monotonic_clock::time_point LogReader::monotonic_start_time() {
  return monotonic_clock::time_point(std::chrono::nanoseconds(
      flatbuffers::GetSizePrefixedRoot<LogFileHeader>(configuration_.data())
          ->monotonic_start_time()));
}

realtime_clock::time_point LogReader::realtime_start_time() {
  return realtime_clock::time_point(std::chrono::nanoseconds(
      flatbuffers::GetSizePrefixedRoot<LogFileHeader>(configuration_.data())
          ->realtime_start_time()));
}

void LogReader::Register(SimulatedEventLoopFactory *event_loop_factory) {
  event_loop_unique_ptr_ = event_loop_factory->MakeEventLoop("log_reader");
  event_loop_factory_ = event_loop_factory;
  // We don't run timing reports when trying to print out logged data, because
  // otherwise we would end up printing out the timing reports themselves...
  // This is only really relevant when we are replaying into a simulation.
  event_loop_unique_ptr_->SkipTimingReport();

  Register(event_loop_unique_ptr_.get());
  event_loop_factory_->RunFor(monotonic_start_time() -
                              event_loop_factory_->monotonic_now());
}

void LogReader::Register(EventLoop *event_loop) {
  event_loop_ = event_loop;

  // Otherwise we replay the timing report and try to resend it...
  event_loop_->SkipTimingReport();

  for (size_t i = 0; i < channels_.size(); ++i) {
    CHECK_EQ(configuration()->channels()->Get(i)->name(),
             event_loop_->configuration()->channels()->Get(i)->name());
    CHECK_EQ(configuration()->channels()->Get(i)->type(),
             event_loop_->configuration()->channels()->Get(i)->type());

    channels_[i].raw_sender = event_loop_->MakeRawSender(
        event_loop_->configuration()->channels()->Get(i));
  }

  timer_handler_ = event_loop_->AddTimer([this]() {
    std::pair<monotonic_clock::time_point, int> oldest_channel_index =
        PopOldestChannel();
    const monotonic_clock::time_point monotonic_now =
        event_loop_->context().monotonic_event_time;
    CHECK(monotonic_now == oldest_channel_index.first)
        << ": Now " << monotonic_now.time_since_epoch().count()
        << " trying to send "
        << oldest_channel_index.first.time_since_epoch().count();

    struct LogReader::ChannelData &channel =
        channels_[oldest_channel_index.second];

    FlatbufferVector<MessageHeader> front = std::move(channel.front());

    if (oldest_channel_index.first > monotonic_start_time() ||
        event_loop_factory_ != nullptr) {
      if (!FLAGS_skip_missing_forwarding_entries ||
          front.message().data() != nullptr) {
        CHECK(front.message().data() != nullptr)
            << ": Got a message without data.  Forwarding entry which was not "
               "matched?  Use --skip_missing_forwarding_entries to ignore "
               "this.";

        // If we have access to the factory, use it to fix the realtime time.
        if (event_loop_factory_ != nullptr) {
          event_loop_factory_->SetRealtimeOffset(
              monotonic_clock::time_point(
                  chrono::nanoseconds(front.message().monotonic_sent_time())),
              realtime_clock::time_point(
                  chrono::nanoseconds(front.message().realtime_sent_time())));
        }

        channel.raw_sender->Send(
            front.message().data()->Data(), front.message().data()->size(),
            monotonic_clock::time_point(
                chrono::nanoseconds(front.message().monotonic_remote_time())),
            realtime_clock::time_point(
                chrono::nanoseconds(front.message().realtime_remote_time())),
            front.message().remote_queue_index());
      }
    } else {
      LOG(WARNING) << "Not sending data from before the start of the log file. "
                   << oldest_channel_index.first.time_since_epoch().count()
                   << " start "
                   << monotonic_start_time().time_since_epoch().count() << " "
                   << FlatbufferToJson(front);
    }
    channel.data.pop_front();

    // Re-push it and update the oldest timestamp.
    if (channel.data.size() != 0) {
      const monotonic_clock::time_point timestamp = monotonic_clock::time_point(
          chrono::nanoseconds(channel.front().message().monotonic_sent_time()));
      PushChannelHeap(timestamp, oldest_channel_index.second);
      channel.oldest_timestamp = timestamp;
    } else {
      channel.oldest_timestamp = monotonic_clock::min_time;
    }

    if (monotonic_now > queue_data_time_) {
      QueueMessages();
    }

    if (channel_heap_.size() != 0) {
      timer_handler_->Setup(oldest_message().first);
    }
  });

  if (channel_heap_.size() > 0u) {
    event_loop_->OnRun(
        [this]() { timer_handler_->Setup(oldest_message().first); });
  }
}

void LogReader::Deregister() {
  for (size_t i = 0; i < channels_.size(); ++i) {
    channels_[i].raw_sender.reset();
  }

  event_loop_factory_ = nullptr;
  event_loop_unique_ptr_.reset();
  event_loop_ = nullptr;
}

void LogReader::EmplaceDataBack(FlatbufferVector<MessageHeader> &&new_data) {
  const monotonic_clock::time_point timestamp = monotonic_clock::time_point(
      chrono::nanoseconds(new_data.message().monotonic_sent_time()));
  const size_t channel_index = new_data.message().channel_index();
  CHECK_LT(channel_index, channels_.size());
  newest_timestamp_ = std::max(newest_timestamp_, timestamp);
  if (channels_[channel_index].data.size() == 0) {
    channels_[channel_index].oldest_timestamp = timestamp;
    PushChannelHeap(timestamp, channel_index);
  }
  channels_[channel_index].data.emplace_back(std::move(new_data));
}

namespace {

bool ChannelHeapCompare(
    const std::pair<monotonic_clock::time_point, int> first,
    const std::pair<monotonic_clock::time_point, int> second) {
  if (first.first > second.first) {
    return true;
  } else if (first.first == second.first) {
    return first.second > second.second;
  } else {
    return false;
  }
}

}  // namespace

void LogReader::PushChannelHeap(monotonic_clock::time_point timestamp,
                                int channel_index) {
  channel_heap_.push_back(std::make_pair(timestamp, channel_index));

  // The default sort puts the newest message first.  Use a custom comparator to
  // put the oldest message first.
  std::push_heap(channel_heap_.begin(), channel_heap_.end(),
                 ChannelHeapCompare);
}

std::pair<monotonic_clock::time_point, int> LogReader::PopOldestChannel() {
  std::pair<monotonic_clock::time_point, int> result = channel_heap_.front();
  std::pop_heap(channel_heap_.begin(), channel_heap_.end(),
                &ChannelHeapCompare);
  channel_heap_.pop_back();
  return result;
}

}  // namespace logger
}  // namespace aos
