#include "aos/events/logging/log_reader.h"

#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <vector>

#include "absl/strings/escaping.h"
#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/logging/uuid.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/multinode_timestamp_filter.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/remote_message_schema.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "flatbuffers/flatbuffers.h"
#include "openssl/sha.h"

DEFINE_bool(skip_missing_forwarding_entries, false,
            "If true, drop any forwarding entries with missing data.  If "
            "false, CHECK.");

DECLARE_bool(timestamps_to_csv);

DEFINE_bool(skip_order_validation, false,
            "If true, ignore any out of orderness in replay");

DEFINE_double(
    time_estimation_buffer_seconds, 2.0,
    "The time to buffer ahead in the log file to accurately reconstruct time.");

namespace aos {
namespace logger {
namespace {

std::string LogFileVectorToString(std::vector<LogFile> log_files) {
  std::stringstream ss;
  for (const auto &f : log_files) {
    ss << f << "\n";
  }
  return ss.str();
}

// Copies the channel, removing the schema as we go.  If new_name is provided,
// it is used instead of the name inside the channel.  If new_type is provided,
// it is used instead of the type in the channel.
flatbuffers::Offset<Channel> CopyChannel(const Channel *c,
                                         std::string_view new_name,
                                         std::string_view new_type,
                                         flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb->CreateSharedString(new_name.empty() ? c->name()->string_view()
                                               : new_name);
  flatbuffers::Offset<flatbuffers::String> type_offset =
      fbb->CreateSharedString(new_type.empty() ? c->type()->str() : new_type);
  flatbuffers::Offset<flatbuffers::String> source_node_offset =
      c->has_source_node() ? fbb->CreateSharedString(c->source_node()->str())
                           : 0;

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Connection>>>
      destination_nodes_offset =
          aos::RecursiveCopyVectorTable(c->destination_nodes(), fbb);

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      logger_nodes_offset = aos::CopyVectorSharedString(c->logger_nodes(), fbb);

  Channel::Builder channel_builder(*fbb);
  channel_builder.add_name(name_offset);
  channel_builder.add_type(type_offset);
  if (c->has_frequency()) {
    channel_builder.add_frequency(c->frequency());
  }
  if (c->has_max_size()) {
    channel_builder.add_max_size(c->max_size());
  }
  if (c->has_num_senders()) {
    channel_builder.add_num_senders(c->num_senders());
  }
  if (c->has_num_watchers()) {
    channel_builder.add_num_watchers(c->num_watchers());
  }
  if (!source_node_offset.IsNull()) {
    channel_builder.add_source_node(source_node_offset);
  }
  if (!destination_nodes_offset.IsNull()) {
    channel_builder.add_destination_nodes(destination_nodes_offset);
  }
  if (c->has_logger()) {
    channel_builder.add_logger(c->logger());
  }
  if (!logger_nodes_offset.IsNull()) {
    channel_builder.add_logger_nodes(logger_nodes_offset);
  }
  if (c->has_read_method()) {
    channel_builder.add_read_method(c->read_method());
  }
  if (c->has_num_readers()) {
    channel_builder.add_num_readers(c->num_readers());
  }
  return channel_builder.Finish();
}

namespace chrono = std::chrono;
using message_bridge::RemoteMessage;
}  // namespace

LogReader::LogReader(std::string_view filename,
                     const Configuration *replay_configuration)
    : LogReader(SortParts({std::string(filename)}), replay_configuration) {}

LogReader::LogReader(std::vector<LogFile> log_files,
                     const Configuration *replay_configuration)
    : log_files_(std::move(log_files)),
      replay_configuration_(replay_configuration) {
  CHECK_GT(log_files_.size(), 0u);
  {
    // Validate that we have the same config everwhere.  This will be true if
    // all the parts were sorted together and the configs match.
    const Configuration *config = nullptr;
    for (const LogFile &log_file : log_files_) {
      if (log_file.config.get() == nullptr) {
        LOG(FATAL) << "Couldn't find a config in " << log_file;
      }
      if (config == nullptr) {
        config = log_file.config.get();
      } else {
        CHECK_EQ(config, log_file.config.get());
      }
    }
  }

  MakeRemappedConfig();

  // Remap all existing remote timestamp channels.  They will be recreated, and
  // the data logged isn't relevant anymore.
  for (const Node *node : configuration::GetNodes(logged_configuration())) {
    std::vector<const Node *> timestamp_logger_nodes =
        configuration::TimestampNodes(logged_configuration(), node);
    for (const Node *remote_node : timestamp_logger_nodes) {
      const std::string channel = absl::StrCat(
          "/aos/remote_timestamps/", remote_node->name()->string_view());
      // See if the log file is an old log with MessageHeader channels in it, or
      // a newer log with RemoteMessage.  If we find an older log, rename the
      // type too along with the name.
      if (HasChannel<MessageHeader>(channel, node)) {
        CHECK(!HasChannel<RemoteMessage>(channel, node))
            << ": Can't have both a MessageHeader and RemoteMessage remote "
               "timestamp channel.";
        // In theory, we should check NOT_LOGGED like RemoteMessage and be more
        // careful about updating the config, but there are fewer and fewer logs
        // with MessageHeader remote messages, so it isn't worth the effort.
        RemapLoggedChannel<MessageHeader>(channel, node, "/original",
                                          "aos.message_bridge.RemoteMessage");
      } else {
        CHECK(HasChannel<RemoteMessage>(channel, node))
            << ": Failed to find {\"name\": \"" << channel << "\", \"type\": \""
            << RemoteMessage::GetFullyQualifiedName() << "\"} for node "
            << node->name()->string_view();
        // Only bother to remap if there's something on the channel.  We can
        // tell if the channel was marked NOT_LOGGED or not.  This makes the
        // config not change un-necesarily when we replay a log with NOT_LOGGED
        // messages.
        if (HasLoggedChannel<RemoteMessage>(channel, node)) {
          RemapLoggedChannel<RemoteMessage>(channel, node);
        }
      }
    }
  }

  if (replay_configuration) {
    CHECK_EQ(configuration::MultiNode(configuration()),
             configuration::MultiNode(replay_configuration))
        << ": Log file and replay config need to both be multi or single "
           "node.";
  }

  if (!configuration::MultiNode(configuration())) {
    states_.emplace_back(std::make_unique<State>(
        std::make_unique<TimestampMapper>(FilterPartsForNode(log_files_, ""))));
  } else {
    if (replay_configuration) {
      CHECK_EQ(logged_configuration()->nodes()->size(),
               replay_configuration->nodes()->size())
          << ": Log file and replay config need to have matching nodes "
             "lists.";
      for (const Node *node : *logged_configuration()->nodes()) {
        if (configuration::GetNode(replay_configuration, node) == nullptr) {
          LOG(FATAL) << "Found node " << FlatbufferToJson(node)
                     << " in logged config that is not present in the replay "
                        "config.";
        }
      }
    }
    states_.resize(configuration()->nodes()->size());
  }
}

LogReader::~LogReader() {
  if (event_loop_factory_unique_ptr_) {
    Deregister();
  } else if (event_loop_factory_ != nullptr) {
    LOG(FATAL) << "Must call Deregister before the SimulatedEventLoopFactory "
                  "is destroyed";
  }
  // Zero out some buffers. It's easy to do use-after-frees on these, so make
  // it more obvious.
  if (remapped_configuration_buffer_) {
    remapped_configuration_buffer_->Wipe();
  }
}

const Configuration *LogReader::logged_configuration() const {
  return log_files_[0].config.get();
}

const Configuration *LogReader::configuration() const {
  return remapped_configuration_;
}

std::vector<const Node *> LogReader::LoggedNodes() const {
  return configuration::GetNodes(logged_configuration());
}

monotonic_clock::time_point LogReader::monotonic_start_time(
    const Node *node) const {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), node)].get();
  CHECK(state != nullptr) << ": Unknown node " << FlatbufferToJson(node);

  return state->monotonic_start_time();
}

realtime_clock::time_point LogReader::realtime_start_time(
    const Node *node) const {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), node)].get();
  CHECK(state != nullptr) << ": Unknown node " << FlatbufferToJson(node);

  return state->realtime_start_time();
}

void LogReader::Register() {
  event_loop_factory_unique_ptr_ =
      std::make_unique<SimulatedEventLoopFactory>(configuration());
  Register(event_loop_factory_unique_ptr_.get());
}

void LogReader::Register(SimulatedEventLoopFactory *event_loop_factory) {
  event_loop_factory_ = event_loop_factory;
  remapped_configuration_ = event_loop_factory_->configuration();
  filters_ =
      std::make_unique<message_bridge::MultiNodeNoncausalOffsetEstimator>(
          event_loop_factory_->configuration(), logged_configuration(),
          FLAGS_skip_order_validation,
          chrono::duration_cast<chrono::nanoseconds>(
              chrono::duration<double>(FLAGS_time_estimation_buffer_seconds)));

  std::vector<TimestampMapper *> timestamp_mappers;
  for (const Node *node : configuration::GetNodes(configuration())) {
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);
    std::vector<LogParts> filtered_parts = FilterPartsForNode(
        log_files_, node != nullptr ? node->name()->string_view() : "");

    // Confirm that all the parts are from the same boot if there are enough
    // parts to not be from the same boot.
    if (filtered_parts.size() > 1u) {
      for (size_t i = 1; i < filtered_parts.size(); ++i) {
        CHECK_EQ(filtered_parts[i].source_boot_uuid,
                 filtered_parts[0].source_boot_uuid)
            << ": Found parts from different boots "
            << LogFileVectorToString(log_files_);
      }
      if (!filtered_parts[0].source_boot_uuid.empty()) {
        event_loop_factory_->GetNodeEventLoopFactory(node)->set_boot_uuid(
            filtered_parts[0].source_boot_uuid);
      }
    }

    states_[node_index] = std::make_unique<State>(
        filtered_parts.size() == 0u
            ? nullptr
            : std::make_unique<TimestampMapper>(std::move(filtered_parts)));
    State *state = states_[node_index].get();
    state->set_event_loop(state->SetNodeEventLoopFactory(
        event_loop_factory_->GetNodeEventLoopFactory(node)));

    state->SetChannelCount(logged_configuration()->channels()->size());
    timestamp_mappers.emplace_back(state->timestamp_mapper());
  }
  filters_->SetTimestampMappers(std::move(timestamp_mappers));

  // Note: this needs to be set before any times are pulled, or we won't observe
  // the timestamps.
  event_loop_factory_->SetTimeConverter(filters_.get());

  for (const Node *node : configuration::GetNodes(configuration())) {
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);
    State *state = states_[node_index].get();
    for (const Node *other_node : configuration::GetNodes(configuration())) {
      const size_t other_node_index =
          configuration::GetNodeIndex(configuration(), other_node);
      State *other_state = states_[other_node_index].get();
      if (other_state != state) {
        state->AddPeer(other_state);
      }
    }
  }

  // Register after making all the State objects so we can build references
  // between them.
  for (const Node *node : configuration::GetNodes(configuration())) {
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);
    State *state = states_[node_index].get();

    Register(state->event_loop());
  }

  if (live_nodes_ == 0) {
    LOG(FATAL)
        << "Don't have logs from any of the nodes in the replay config--are "
           "you sure that the replay config matches the original config?";
  }

  filters_->CheckGraph();

  for (std::unique_ptr<State> &state : states_) {
    state->SeedSortedMessages();
  }

  // We want to start the log file at the last start time of the log files
  // from all the nodes.  Compute how long each node's simulation needs to run
  // to move time to this point.
  distributed_clock::time_point start_time = distributed_clock::min_time;

  // TODO(austin): We want an "OnStart" callback for each node rather than
  // running until the last node.

  for (std::unique_ptr<State> &state : states_) {
    VLOG(1) << "Start time is " << state->monotonic_start_time() << " for node "
            << MaybeNodeName(state->event_loop()->node()) << "now "
            << state->monotonic_now();
    if (state->monotonic_start_time() == monotonic_clock::min_time) {
      continue;
    }
    // And start computing the start time on the distributed clock now that
    // that works.
    start_time = std::max(
        start_time, state->ToDistributedClock(state->monotonic_start_time()));
  }

  // TODO(austin): If a node doesn't have a start time, we might not queue
  // enough.  If this happens, we'll explode with a frozen error eventually.

  CHECK_GE(start_time, distributed_clock::epoch())
      << ": Hmm, we have a node starting before the start of time.  Offset "
         "everything.";

  // Forwarding is tracked per channel.  If it is enabled, we want to turn it
  // off.  Otherwise messages replayed will get forwarded across to the other
  // nodes, and also replayed on the other nodes.  This may not satisfy all
  // our users, but it'll start the discussion.
  if (configuration::MultiNode(event_loop_factory_->configuration())) {
    for (size_t i = 0; i < logged_configuration()->channels()->size(); ++i) {
      const Channel *channel = logged_configuration()->channels()->Get(i);
      const Node *node = configuration::GetNode(
          configuration(), channel->source_node()->string_view());

      State *state =
          states_[configuration::GetNodeIndex(configuration(), node)].get();

      const Channel *remapped_channel =
          RemapChannel(state->event_loop(), channel);

      event_loop_factory_->DisableForwarding(remapped_channel);
    }

    // If we are replaying a log, we don't want a bunch of redundant messages
    // from both the real message bridge and simulated message bridge.
    event_loop_factory_->DisableStatistics();
  }

  // While we are starting the system up, we might be relying on matching data
  // to timestamps on log files where the timestamp log file starts before the
  // data.  In this case, it is reasonable to expect missing data.
  {
    const bool prior_ignore_missing_data = ignore_missing_data_;
    ignore_missing_data_ = true;
    VLOG(1) << "Running until " << start_time << " in Register";
    event_loop_factory_->RunFor(start_time.time_since_epoch());
    VLOG(1) << "At start time";
    // Now that we are running for real, missing data means that the log file is
    // corrupted or went wrong.
    ignore_missing_data_ = prior_ignore_missing_data;
  }

  for (std::unique_ptr<State> &state : states_) {
    // Make the RT clock be correct before handing it to the user.
    if (state->realtime_start_time() != realtime_clock::min_time) {
      state->SetRealtimeOffset(state->monotonic_start_time(),
                               state->realtime_start_time());
    }
    VLOG(1) << "Start time is " << state->monotonic_start_time() << " for node "
            << MaybeNodeName(state->event_loop()->node()) << "now "
            << state->monotonic_now();
  }

  if (FLAGS_timestamps_to_csv) {
    filters_->Start(event_loop_factory);
  }
}

message_bridge::NoncausalOffsetEstimator *LogReader::GetFilter(
    const Node *node_a, const Node *node_b) {
  if (filters_) {
    return filters_->GetFilter(node_a, node_b);
  }
  return nullptr;
}

void LogReader::Register(EventLoop *event_loop) {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), event_loop->node())]
          .get();

  state->set_event_loop(event_loop);

  // We don't run timing reports when trying to print out logged data, because
  // otherwise we would end up printing out the timing reports themselves...
  // This is only really relevant when we are replaying into a simulation.
  event_loop->SkipTimingReport();
  event_loop->SkipAosLog();

  for (size_t logged_channel_index = 0;
       logged_channel_index < logged_configuration()->channels()->size();
       ++logged_channel_index) {
    const Channel *channel = RemapChannel(
        event_loop,
        logged_configuration()->channels()->Get(logged_channel_index));

    if (channel->logger() == LoggerConfig::NOT_LOGGED) {
      continue;
    }

    message_bridge::NoncausalOffsetEstimator *filter = nullptr;
    RemoteMessageSender *remote_timestamp_sender = nullptr;

    State *source_state = nullptr;

    if (!configuration::ChannelIsSendableOnNode(channel, event_loop->node()) &&
        configuration::ChannelIsReadableOnNode(channel, event_loop->node())) {
      // We've got a message which is being forwarded to this node.
      const Node *source_node = configuration::GetNode(
          event_loop->configuration(), channel->source_node()->string_view());
      filter = GetFilter(event_loop->node(), source_node);

      // Delivery timestamps are supposed to be logged back on the source node.
      // Configure remote timestamps to be sent.
      const bool delivery_time_is_logged =
          configuration::ConnectionDeliveryTimeIsLoggedOnNode(
              channel, event_loop->node(), source_node);

      source_state =
          states_[configuration::GetNodeIndex(configuration(), source_node)]
              .get();

      if (delivery_time_is_logged) {
        remote_timestamp_sender =
            source_state->RemoteTimestampSender(event_loop->node());
      }
    }

    state->SetChannel(
        logged_channel_index,
        configuration::ChannelIndex(event_loop->configuration(), channel),
        event_loop->MakeRawSender(channel), filter, remote_timestamp_sender,
        source_state);
  }

  // If we didn't find any log files with data in them, we won't ever get a
  // callback or be live.  So skip the rest of the setup.
  if (state->OldestMessageTime() == monotonic_clock::max_time) {
    return;
  }

  state->set_timer_handler(event_loop->AddTimer([this, state]() {
    VLOG(1) << "Starting sending " << MaybeNodeName(state->event_loop()->node())
            << "at " << state->event_loop()->context().monotonic_event_time
            << " now " << state->monotonic_now();
    if (state->OldestMessageTime() == monotonic_clock::max_time) {
      --live_nodes_;
      VLOG(1) << MaybeNodeName(state->event_loop()->node()) << "Node down!";
      if (exit_on_finish_ && live_nodes_ == 0) {
        event_loop_factory_->Exit();
      }
      return;
    }

    TimestampedMessage timestamped_message = state->PopOldest();

    const monotonic_clock::time_point monotonic_now =
        state->event_loop()->context().monotonic_event_time;
    if (!FLAGS_skip_order_validation) {
      CHECK(monotonic_now == timestamped_message.monotonic_event_time)
          << ": " << FlatbufferToJson(state->event_loop()->node()) << " Now "
          << monotonic_now << " trying to send "
          << timestamped_message.monotonic_event_time << " failure "
          << state->DebugString();
    } else if (monotonic_now != timestamped_message.monotonic_event_time) {
      LOG(WARNING) << "Check failed: monotonic_now == "
                      "timestamped_message.monotonic_event_time) ("
                   << monotonic_now << " vs. "
                   << timestamped_message.monotonic_event_time
                   << "): " << FlatbufferToJson(state->event_loop()->node())
                   << " Now " << monotonic_now << " trying to send "
                   << timestamped_message.monotonic_event_time << " failure "
                   << state->DebugString();
    }

    if (timestamped_message.monotonic_event_time >
            state->monotonic_start_time() ||
        event_loop_factory_ != nullptr) {
      if (timestamped_message.data.span().size() != 0u) {
        if (timestamped_message.monotonic_remote_time !=
            monotonic_clock::min_time) {
          // Confirm that the message was sent on the sending node before the
          // destination node (this node).  As a proxy, do this by making sure
          // that time on the source node is past when the message was sent.
          //
          // TODO(austin): <= means that the cause message (which we know) could
          // happen after the effect even though we know they are at the same
          // time.  I doubt anyone will notice for a bit, but we should really
          // fix that.
          if (!FLAGS_skip_order_validation) {
            CHECK_LE(
                timestamped_message.monotonic_remote_time,
                state->monotonic_remote_now(timestamped_message.channel_index))
                << state->event_loop()->node()->name()->string_view() << " to "
                << state->remote_node(timestamped_message.channel_index)
                       ->name()
                       ->string_view()
                << " while trying to send a message on "
                << configuration::CleanedChannelToString(
                       logged_configuration()->channels()->Get(
                           timestamped_message.channel_index))
                << " " << state->DebugString();
          } else if (timestamped_message.monotonic_remote_time >
                     state->monotonic_remote_now(
                         timestamped_message.channel_index)) {
            LOG(WARNING)
                << "Check failed: timestamped_message.monotonic_remote_time < "
                   "state->monotonic_remote_now(timestamped_message.channel_"
                   "index) ("
                << timestamped_message.monotonic_remote_time << " vs. "
                << state->monotonic_remote_now(
                       timestamped_message.channel_index)
                << ") " << state->event_loop()->node()->name()->string_view()
                << " to "
                << state->remote_node(timestamped_message.channel_index)
                       ->name()
                       ->string_view()
                << " currently " << timestamped_message.monotonic_event_time
                << " ("
                << state->ToDistributedClock(
                       timestamped_message.monotonic_event_time)
                << ") remote event time "
                << timestamped_message.monotonic_remote_time << " ("
                << state->RemoteToDistributedClock(
                       timestamped_message.channel_index,
                       timestamped_message.monotonic_remote_time)
                << ") " << state->DebugString();
          }
        }

        // If we have access to the factory, use it to fix the realtime time.
        state->SetRealtimeOffset(timestamped_message.monotonic_event_time,
                                 timestamped_message.realtime_event_time);

        VLOG(1) << MaybeNodeName(state->event_loop()->node()) << "Sending "
                << timestamped_message.monotonic_event_time;
        // TODO(austin): std::move channel_data in and make that efficient in
        // simulation.
        state->Send(std::move(timestamped_message));
      } else if (!ignore_missing_data_ &&
                 // When starting up, we can have data which was sent before the
                 // log starts, but the timestamp was after the log starts. This
                 // is unreasonable to avoid, so ignore the missing data.
                 timestamped_message.monotonic_remote_time >=
                     state->monotonic_remote_start_time(
                         timestamped_message.channel_index) &&
                 !FLAGS_skip_missing_forwarding_entries) {
        // We've found a timestamp without data that we expect to have data for.
        // This likely means that we are at the end of the log file.  Record it
        // and CHECK that in the rest of the log file, we don't find any more
        // data on that channel.  Not all channels will end at the same point in
        // time since they can be in different files.
        VLOG(1) << "Found the last message on channel "
                << timestamped_message.channel_index;

        // Vector storing if we've seen a nullptr message or not per channel.
        std::vector<bool> last_message;
        last_message.resize(logged_configuration()->channels()->size(), false);

        last_message[timestamped_message.channel_index] = true;

        // Now that we found the end of one channel, artificially stop the
        // rest.  It is confusing when part of your data gets replayed but not
        // all.  Read the rest of the messages and drop them on the floor while
        // doing some basic validation.
        while (state->OldestMessageTime() != monotonic_clock::max_time) {
          TimestampedMessage next = state->PopOldest();
          // Make sure that once we have seen the last message on a channel,
          // data doesn't start back up again.  If the user wants to play
          // through events like this, they can set
          // --skip_missing_forwarding_entries or ignore_missing_data_.
          CHECK_LT(next.channel_index, last_message.size());
          if (next.data.span().size() == 0u) {
            last_message[next.channel_index] = true;
          } else {
            if (last_message[next.channel_index]) {
              LOG(FATAL)
                  << "Found missing data in the middle of the log file on "
                     "channel "
                  << next.channel_index << " Last "
                  << last_message[next.channel_index] << state->DebugString();
            }
          }
        }
      }
    } else {
      LOG(WARNING)
          << "Not sending data from before the start of the log file. "
          << timestamped_message.monotonic_event_time.time_since_epoch().count()
          << " start " << monotonic_start_time().time_since_epoch().count()
          << " "
          << FlatbufferToJson(timestamped_message.data,
                              {.multi_line = false, .max_vector_size = 100});
    }

    const monotonic_clock::time_point next_time = state->OldestMessageTime();
    if (next_time != monotonic_clock::max_time) {
      VLOG(1) << "Scheduling " << MaybeNodeName(state->event_loop()->node())
              << "wakeup for " << next_time << "("
              << state->ToDistributedClock(next_time)
              << " distributed), now is " << state->monotonic_now();
      state->Setup(next_time);
    } else {
      VLOG(1) << MaybeNodeName(state->event_loop()->node())
              << "No next message, scheduling shutdown";
      // Set a timer up immediately after now to die. If we don't do this,
      // then the senders waiting on the message we just read will never get
      // called.
      if (event_loop_factory_ != nullptr) {
        state->Setup(monotonic_now + event_loop_factory_->send_delay() +
                     std::chrono::nanoseconds(1));
      }
    }

    VLOG(1) << MaybeNodeName(state->event_loop()->node()) << "Done sending at "
            << state->event_loop()->context().monotonic_event_time << " now "
            << state->monotonic_now();
  }));

  ++live_nodes_;

  if (state->OldestMessageTime() != monotonic_clock::max_time) {
    event_loop->OnRun([state]() { state->Setup(state->OldestMessageTime()); });
  }
}

void LogReader::Deregister() {
  // Make sure that things get destroyed in the correct order, rather than
  // relying on getting the order correct in the class definition.
  for (std::unique_ptr<State> &state : states_) {
    state->Deregister();
  }

  event_loop_factory_unique_ptr_.reset();
  event_loop_factory_ = nullptr;
}

void LogReader::RemapLoggedChannel(std::string_view name, std::string_view type,
                                   std::string_view add_prefix,
                                   std::string_view new_type) {
  for (size_t ii = 0; ii < logged_configuration()->channels()->size(); ++ii) {
    const Channel *const channel = logged_configuration()->channels()->Get(ii);
    if (channel->name()->str() == name &&
        channel->type()->string_view() == type) {
      CHECK_EQ(0u, remapped_channels_.count(ii))
          << "Already remapped channel "
          << configuration::CleanedChannelToString(channel);
      RemappedChannel remapped_channel;
      remapped_channel.remapped_name =
          std::string(add_prefix) + std::string(name);
      remapped_channel.new_type = new_type;
      remapped_channels_[ii] = std::move(remapped_channel);
      VLOG(1) << "Remapping channel "
              << configuration::CleanedChannelToString(channel)
              << " to have name " << remapped_channels_[ii].remapped_name;
      MakeRemappedConfig();
      return;
    }
  }
  LOG(FATAL) << "Unabled to locate channel with name " << name << " and type "
             << type;
}

void LogReader::RemapLoggedChannel(std::string_view name, std::string_view type,
                                   const Node *node,
                                   std::string_view add_prefix,
                                   std::string_view new_type) {
  VLOG(1) << "Node is " << aos::FlatbufferToJson(node);
  const Channel *remapped_channel =
      configuration::GetChannel(logged_configuration(), name, type, "", node);
  CHECK(remapped_channel != nullptr) << ": Failed to find {\"name\": \"" << name
                                     << "\", \"type\": \"" << type << "\"}";
  VLOG(1) << "Original {\"name\": \"" << name << "\", \"type\": \"" << type
          << "\"}";
  VLOG(1) << "Remapped "
          << aos::configuration::StrippedChannelToString(remapped_channel);

  // We want to make /spray on node 0 go to /0/spray by snooping the maps.  And
  // we want it to degrade if the heuristics fail to just work.
  //
  // The easiest way to do this is going to be incredibly specific and verbose.
  // Look up /spray, to /0/spray.  Then, prefix the result with /original to get
  // /original/0/spray.  Then, create a map from /original/spray to
  // /original/0/spray for just the type we were asked for.
  if (name != remapped_channel->name()->string_view()) {
    MapT new_map;
    new_map.match = std::make_unique<ChannelT>();
    new_map.match->name = absl::StrCat(add_prefix, name);
    new_map.match->type = type;
    if (node != nullptr) {
      new_map.match->source_node = node->name()->str();
    }
    new_map.rename = std::make_unique<ChannelT>();
    new_map.rename->name =
        absl::StrCat(add_prefix, remapped_channel->name()->string_view());
    maps_.emplace_back(std::move(new_map));
  }

  const size_t channel_index =
      configuration::ChannelIndex(logged_configuration(), remapped_channel);
  CHECK_EQ(0u, remapped_channels_.count(channel_index))
      << "Already remapped channel "
      << configuration::CleanedChannelToString(remapped_channel);

  RemappedChannel remapped_channel_struct;
  remapped_channel_struct.remapped_name =
      std::string(add_prefix) +
      std::string(remapped_channel->name()->string_view());
  remapped_channel_struct.new_type = new_type;
  remapped_channels_[channel_index] = std::move(remapped_channel_struct);
  MakeRemappedConfig();
}

void LogReader::MakeRemappedConfig() {
  for (std::unique_ptr<State> &state : states_) {
    if (state) {
      CHECK(!state->event_loop())
          << ": Can't change the mapping after the events are scheduled.";
    }
  }

  // If no remapping occurred and we are using the original config, then there
  // is nothing interesting to do here.
  if (remapped_channels_.empty() && replay_configuration_ == nullptr) {
    remapped_configuration_ = logged_configuration();
    return;
  }
  // Config to copy Channel definitions from. Use the specified
  // replay_configuration_ if it has been provided.
  const Configuration *const base_config = replay_configuration_ == nullptr
                                               ? logged_configuration()
                                               : replay_configuration_;

  // Create a config with all the channels, but un-sorted/merged.  Collect up
  // the schemas while we do this.  Call MergeConfiguration to sort everything,
  // and then merge it all in together.

  // This is the builder that we use for the config containing all the new
  // channels.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  std::vector<flatbuffers::Offset<Channel>> channel_offsets;

  CHECK_EQ(Channel::MiniReflectTypeTable()->num_elems, 13u)
      << ": Merging logic needs to be updated when the number of channel "
         "fields changes.";

  // List of schemas.
  std::map<std::string_view, FlatbufferVector<reflection::Schema>> schema_map;
  // Make sure our new RemoteMessage schema is in there for old logs without it.
  schema_map.insert(std::make_pair(
      RemoteMessage::GetFullyQualifiedName(),
      FlatbufferVector<reflection::Schema>(FlatbufferSpan<reflection::Schema>(
          message_bridge::RemoteMessageSchema()))));

  // Reconstruct the remapped channels.
  for (auto &pair : remapped_channels_) {
    const Channel *const c = CHECK_NOTNULL(configuration::GetChannel(
        base_config, logged_configuration()->channels()->Get(pair.first), "",
        nullptr));
    channel_offsets.emplace_back(
        CopyChannel(c, pair.second.remapped_name, "", &fbb));
  }

  // Now reconstruct the original channels, translating types as needed
  for (const Channel *c : *base_config->channels()) {
    // Search for a mapping channel.
    std::string_view new_type = "";
    for (auto &pair : remapped_channels_) {
      const Channel *const remapped_channel =
          logged_configuration()->channels()->Get(pair.first);
      if (remapped_channel->name()->string_view() == c->name()->string_view() &&
          remapped_channel->type()->string_view() == c->type()->string_view()) {
        new_type = pair.second.new_type;
        break;
      }
    }

    // Copy everything over.
    channel_offsets.emplace_back(CopyChannel(c, "", new_type, &fbb));

    // Add the schema if it doesn't exist.
    if (schema_map.find(c->type()->string_view()) == schema_map.end()) {
      CHECK(c->has_schema());
      schema_map.insert(std::make_pair(c->type()->string_view(),
                                       RecursiveCopyFlatBuffer(c->schema())));
    }
  }

  // The MergeConfiguration API takes a vector, not a map.  Convert.
  std::vector<FlatbufferVector<reflection::Schema>> schemas;
  while (!schema_map.empty()) {
    schemas.emplace_back(std::move(schema_map.begin()->second));
    schema_map.erase(schema_map.begin());
  }

  // Create the Configuration containing the new channels that we want to add.
  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset =
          channel_offsets.empty() ? 0 : fbb.CreateVector(channel_offsets);

  // Copy over the old maps.
  std::vector<flatbuffers::Offset<Map>> map_offsets;
  if (base_config->maps()) {
    for (const Map *map : *base_config->maps()) {
      map_offsets.emplace_back(aos::RecursiveCopyFlatBuffer(map, &fbb));
    }
  }

  // Now create the new maps.  These are second so they take effect first.
  for (const MapT &map : maps_) {
    const flatbuffers::Offset<flatbuffers::String> match_name_offset =
        fbb.CreateString(map.match->name);
    const flatbuffers::Offset<flatbuffers::String> match_type_offset =
        fbb.CreateString(map.match->type);
    const flatbuffers::Offset<flatbuffers::String> rename_name_offset =
        fbb.CreateString(map.rename->name);
    flatbuffers::Offset<flatbuffers::String> match_source_node_offset;
    if (!map.match->source_node.empty()) {
      match_source_node_offset = fbb.CreateString(map.match->source_node);
    }
    Channel::Builder match_builder(fbb);
    match_builder.add_name(match_name_offset);
    match_builder.add_type(match_type_offset);
    if (!map.match->source_node.empty()) {
      match_builder.add_source_node(match_source_node_offset);
    }
    const flatbuffers::Offset<Channel> match_offset = match_builder.Finish();

    Channel::Builder rename_builder(fbb);
    rename_builder.add_name(rename_name_offset);
    const flatbuffers::Offset<Channel> rename_offset = rename_builder.Finish();

    Map::Builder map_builder(fbb);
    map_builder.add_match(match_offset);
    map_builder.add_rename(rename_offset);
    map_offsets.emplace_back(map_builder.Finish());
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Map>>>
      maps_offsets = map_offsets.empty() ? 0 : fbb.CreateVector(map_offsets);

  // And copy everything else over.
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Node>>>
      nodes_offset = aos::RecursiveCopyVectorTable(base_config->nodes(), &fbb);

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Application>>>
      applications_offset =
          aos::RecursiveCopyVectorTable(base_config->applications(), &fbb);

  // Now insert everything else in unmodified.
  ConfigurationBuilder configuration_builder(fbb);
  if (!channels_offset.IsNull()) {
    configuration_builder.add_channels(channels_offset);
  }
  if (!maps_offsets.IsNull()) {
    configuration_builder.add_maps(maps_offsets);
  }
  if (!nodes_offset.IsNull()) {
    configuration_builder.add_nodes(nodes_offset);
  }
  if (!applications_offset.IsNull()) {
    configuration_builder.add_applications(applications_offset);
  }

  if (base_config->has_channel_storage_duration()) {
    configuration_builder.add_channel_storage_duration(
        base_config->channel_storage_duration());
  }

  CHECK_EQ(Configuration::MiniReflectTypeTable()->num_elems, 6u)
      << ": Merging logic needs to be updated when the number of configuration "
         "fields changes.";

  fbb.Finish(configuration_builder.Finish());

  // Clean it up and return it!  By using MergeConfiguration here, we'll
  // actually get a deduplicated config for free too.
  FlatbufferDetachedBuffer<Configuration> new_merged_config =
      configuration::MergeConfiguration(
          FlatbufferDetachedBuffer<Configuration>(fbb.Release()));

  remapped_configuration_buffer_ =
      std::make_unique<FlatbufferDetachedBuffer<Configuration>>(
          configuration::MergeConfiguration(new_merged_config, schemas));

  remapped_configuration_ = &remapped_configuration_buffer_->message();

  // TODO(austin): Lazily re-build to save CPU?
}

const Channel *LogReader::RemapChannel(const EventLoop *event_loop,
                                       const Channel *channel) {
  std::string_view channel_name = channel->name()->string_view();
  std::string_view channel_type = channel->type()->string_view();
  const int channel_index =
      configuration::ChannelIndex(logged_configuration(), channel);
  // If the channel is remapped, find the correct channel name to use.
  if (remapped_channels_.count(channel_index) > 0) {
    VLOG(3) << "Got remapped channel on "
            << configuration::CleanedChannelToString(channel);
    channel_name = remapped_channels_[channel_index].remapped_name;
  }

  VLOG(2) << "Going to remap channel " << channel_name << " " << channel_type;
  const Channel *remapped_channel = configuration::GetChannel(
      event_loop->configuration(), channel_name, channel_type,
      event_loop->name(), event_loop->node());

  CHECK(remapped_channel != nullptr)
      << ": Unable to send {\"name\": \"" << channel_name << "\", \"type\": \""
      << channel_type << "\"} because it is not in the provided configuration.";

  return remapped_channel;
}

LogReader::State::State(std::unique_ptr<TimestampMapper> timestamp_mapper)
    : timestamp_mapper_(std::move(timestamp_mapper)) {}

void LogReader::State::AddPeer(State *peer) {
  if (timestamp_mapper_ && peer->timestamp_mapper_) {
    timestamp_mapper_->AddPeer(peer->timestamp_mapper_.get());
  }
}

EventLoop *LogReader::State::SetNodeEventLoopFactory(
    NodeEventLoopFactory *node_event_loop_factory) {
  node_event_loop_factory_ = node_event_loop_factory;
  event_loop_unique_ptr_ =
      node_event_loop_factory_->MakeEventLoop("log_reader");
  return event_loop_unique_ptr_.get();
}

void LogReader::State::SetChannelCount(size_t count) {
  channels_.resize(count);
  remote_timestamp_senders_.resize(count);
  filters_.resize(count);
  channel_source_state_.resize(count);
  factory_channel_index_.resize(count);
  queue_index_map_.resize(count);
}

void LogReader::State::SetChannel(
    size_t logged_channel_index, size_t factory_channel_index,
    std::unique_ptr<RawSender> sender,
    message_bridge::NoncausalOffsetEstimator *filter,
    RemoteMessageSender *remote_timestamp_sender, State *source_state) {
  channels_[logged_channel_index] = std::move(sender);
  filters_[logged_channel_index] = filter;
  remote_timestamp_senders_[logged_channel_index] = remote_timestamp_sender;

  if (source_state) {
    channel_source_state_[logged_channel_index] = source_state;

    if (remote_timestamp_sender != nullptr) {
      source_state->queue_index_map_[logged_channel_index] =
          std::make_unique<std::vector<State::ContiguousSentTimestamp>>();
    }
  }

  factory_channel_index_[logged_channel_index] = factory_channel_index;
}

bool LogReader::State::Send(const TimestampedMessage &timestamped_message) {
  aos::RawSender *sender = channels_[timestamped_message.channel_index].get();
  uint32_t remote_queue_index = 0xffffffff;

  if (remote_timestamp_senders_[timestamped_message.channel_index] != nullptr) {
    std::vector<ContiguousSentTimestamp> *queue_index_map = CHECK_NOTNULL(
        CHECK_NOTNULL(channel_source_state_[timestamped_message.channel_index])
            ->queue_index_map_[timestamped_message.channel_index]
            .get());

    struct SentTimestamp {
      monotonic_clock::time_point monotonic_event_time;
      uint32_t queue_index;
    } search;

    search.monotonic_event_time = timestamped_message.monotonic_remote_time;
    search.queue_index = timestamped_message.remote_queue_index;

    // Find the sent time if available.
    auto element = std::lower_bound(
        queue_index_map->begin(), queue_index_map->end(), search,
        [](ContiguousSentTimestamp a, SentTimestamp b) {
          if (a.ending_monotonic_event_time < b.monotonic_event_time) {
            return true;
          }
          if (a.starting_monotonic_event_time > b.monotonic_event_time) {
            return false;
          }

          if (a.ending_queue_index < b.queue_index) {
            return true;
          }
          if (a.starting_queue_index >= b.queue_index) {
            return false;
          }

          // If it isn't clearly below or above, it is below.  Since we return
          // the last element <, this will return a match.
          return false;
        });

    // TODO(austin): Be a bit more principled here, but we will want to do that
    // after the logger rewrite.  We hit this when one node finishes, but the
    // other node isn't done yet.  So there is no send time, but there is a
    // receive time.
    if (element != queue_index_map->end()) {
      CHECK_GE(timestamped_message.monotonic_remote_time,
               element->starting_monotonic_event_time);
      CHECK_LE(timestamped_message.monotonic_remote_time,
               element->ending_monotonic_event_time);
      CHECK_GE(timestamped_message.remote_queue_index,
               element->starting_queue_index);
      CHECK_LE(timestamped_message.remote_queue_index,
               element->ending_queue_index);

      remote_queue_index = timestamped_message.remote_queue_index +
                           element->actual_queue_index -
                           element->starting_queue_index;
    } else {
      VLOG(1) << "No timestamp match in the map.";
    }
  }

  // Send!  Use the replayed queue index here instead of the logged queue index
  // for the remote queue index.  This makes re-logging work.
  const bool sent = sender->Send(
      timestamped_message.data.message().data()->Data(),
      timestamped_message.data.message().data()->size(),
      timestamped_message.monotonic_remote_time,
      timestamped_message.realtime_remote_time, remote_queue_index);
  if (!sent) return false;

  if (queue_index_map_[timestamped_message.channel_index]) {
    if (queue_index_map_[timestamped_message.channel_index]->empty()) {
      // Nothing here, start a range with 0 length.
      ContiguousSentTimestamp timestamp;
      timestamp.starting_monotonic_event_time =
          timestamp.ending_monotonic_event_time =
              timestamped_message.monotonic_event_time;
      timestamp.starting_queue_index = timestamp.ending_queue_index =
          timestamped_message.queue_index;
      timestamp.actual_queue_index = sender->sent_queue_index();
      queue_index_map_[timestamped_message.channel_index]->emplace_back(
          timestamp);
    } else {
      // We've got something.  See if the next timestamp is still contiguous. If
      // so, grow it.
      ContiguousSentTimestamp *back =
          &queue_index_map_[timestamped_message.channel_index]->back();
      if ((back->starting_queue_index - back->actual_queue_index) ==
          (timestamped_message.queue_index - sender->sent_queue_index())) {
        back->ending_queue_index = timestamped_message.queue_index;
        back->ending_monotonic_event_time =
            timestamped_message.monotonic_event_time;
      } else {
        // Otherwise, make a new one.
        ContiguousSentTimestamp timestamp;
        timestamp.starting_monotonic_event_time =
            timestamp.ending_monotonic_event_time =
                timestamped_message.monotonic_event_time;
        timestamp.starting_queue_index = timestamp.ending_queue_index =
            timestamped_message.queue_index;
        timestamp.actual_queue_index = sender->sent_queue_index();
        queue_index_map_[timestamped_message.channel_index]->emplace_back(
            timestamp);
      }
    }

    // TODO(austin): Should we prune the map?  On a many day log, I only saw the
    // queue index diverge a couple of elements, which would be a very small
    // map.
  } else if (remote_timestamp_senders_[timestamped_message.channel_index] !=
             nullptr) {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    flatbuffers::Offset<flatbuffers::String> boot_uuid_offset =
        fbb.CreateString(event_loop_->boot_uuid().string_view());

    RemoteMessage::Builder message_header_builder(fbb);

    message_header_builder.add_channel_index(
        factory_channel_index_[timestamped_message.channel_index]);

    // Swap the remote and sent metrics.  They are from the sender's
    // perspective, not the receiver's perspective.
    message_header_builder.add_monotonic_sent_time(
        sender->monotonic_sent_time().time_since_epoch().count());
    message_header_builder.add_realtime_sent_time(
        sender->realtime_sent_time().time_since_epoch().count());
    message_header_builder.add_queue_index(sender->sent_queue_index());

    message_header_builder.add_monotonic_remote_time(
        timestamped_message.monotonic_remote_time.time_since_epoch().count());
    message_header_builder.add_realtime_remote_time(
        timestamped_message.realtime_remote_time.time_since_epoch().count());

    message_header_builder.add_remote_queue_index(remote_queue_index);
    message_header_builder.add_boot_uuid(boot_uuid_offset);

    fbb.Finish(message_header_builder.Finish());

    remote_timestamp_senders_[timestamped_message.channel_index]->Send(
        FlatbufferDetachedBuffer<RemoteMessage>(fbb.Release()),
        timestamped_message.monotonic_timestamp_time);
  }

  return true;
}

LogReader::RemoteMessageSender::RemoteMessageSender(
    aos::Sender<message_bridge::RemoteMessage> sender, EventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(std::move(sender)),
      timer_(event_loop->AddTimer([this]() { SendTimestamp(); })) {}

void LogReader::RemoteMessageSender::ScheduleTimestamp() {
  if (remote_timestamps_.empty()) {
    CHECK_NOTNULL(timer_);
    timer_->Disable();
    scheduled_time_ = monotonic_clock::min_time;
    return;
  }

  if (scheduled_time_ != remote_timestamps_.front().monotonic_timestamp_time) {
    CHECK_NOTNULL(timer_);
    timer_->Setup(remote_timestamps_.front().monotonic_timestamp_time);
    scheduled_time_ = remote_timestamps_.front().monotonic_timestamp_time;
    CHECK_GE(scheduled_time_, event_loop_->monotonic_now())
        << event_loop_->node()->name()->string_view();
  }
}

void LogReader::RemoteMessageSender::Send(
    FlatbufferDetachedBuffer<RemoteMessage> remote_message,
    monotonic_clock::time_point monotonic_timestamp_time) {
  // There are 2 cases.  Either we have a monotonic_timestamp_time and need to
  // resend the timestamp at the correct time, or we don't and can send it
  // immediately.
  if (monotonic_timestamp_time == monotonic_clock::min_time) {
    CHECK(remote_timestamps_.empty())
        << ": Unsupported mix of timestamps and no timestamps.";
    sender_.Send(std::move(remote_message));
  } else {
    remote_timestamps_.emplace(
        std::upper_bound(
            remote_timestamps_.begin(), remote_timestamps_.end(),
            monotonic_timestamp_time,
            [](const aos::monotonic_clock::time_point monotonic_timestamp_time,
               const Timestamp &timestamp) {
              return monotonic_timestamp_time <
                     timestamp.monotonic_timestamp_time;
            }),
        std::move(remote_message), monotonic_timestamp_time);
    ScheduleTimestamp();
  }
}

void LogReader::RemoteMessageSender::SendTimestamp() {
  CHECK_EQ(event_loop_->context().monotonic_event_time, scheduled_time_)
      << event_loop_->node()->name()->string_view();
  CHECK(!remote_timestamps_.empty());

  // Send out all timestamps at the currently scheduled time.
  while (remote_timestamps_.front().monotonic_timestamp_time ==
         scheduled_time_) {
    sender_.Send(std::move(remote_timestamps_.front().remote_message));
    remote_timestamps_.pop_front();
    if (remote_timestamps_.empty()) {
      break;
    }
  }
  scheduled_time_ = monotonic_clock::min_time;

  ScheduleTimestamp();
}

LogReader::RemoteMessageSender *LogReader::State::RemoteTimestampSender(
    const Node *delivered_node) {
  auto sender = remote_timestamp_senders_map_.find(delivered_node);

  if (sender == remote_timestamp_senders_map_.end()) {
    sender =
        remote_timestamp_senders_map_
            .emplace(delivered_node,
                     std::make_unique<RemoteMessageSender>(
                         event_loop()->MakeSender<RemoteMessage>(absl::StrCat(
                             "/aos/remote_timestamps/",
                             delivered_node->name()->string_view())),
                         event_loop()))
            .first;
  }

  return sender->second.get();
}

TimestampedMessage LogReader::State::PopOldest() {
  CHECK(timestamp_mapper_ != nullptr);
  TimestampedMessage *result_ptr = timestamp_mapper_->Front();
  CHECK(result_ptr != nullptr);

  TimestampedMessage result = std::move(*result_ptr);

  VLOG(2) << MaybeNodeName(event_loop_->node()) << "PopOldest Popping "
          << result.monotonic_event_time;
  timestamp_mapper_->PopFront();
  SeedSortedMessages();

  if (result.monotonic_remote_time != monotonic_clock::min_time) {
    message_bridge::NoncausalOffsetEstimator *filter =
        filters_[result.channel_index];
    CHECK(filter != nullptr);

    // TODO(austin): We probably want to push this down into the timestamp
    // mapper directly.
    filter->Pop(event_loop_->node(), event_loop_->monotonic_now());
  }
  VLOG(1) << "Popped " << result
          << configuration::CleanedChannelToString(
                 event_loop_->configuration()->channels()->Get(
                     factory_channel_index_[result.channel_index]));
  return result;
}

monotonic_clock::time_point LogReader::State::OldestMessageTime() const {
  if (timestamp_mapper_ == nullptr) {
    return monotonic_clock::max_time;
  }
  TimestampedMessage *result_ptr = timestamp_mapper_->Front();
  if (result_ptr == nullptr) {
    return monotonic_clock::max_time;
  }
  VLOG(2) << MaybeNodeName(event_loop_->node()) << "oldest message at "
          << result_ptr->monotonic_event_time;
  return result_ptr->monotonic_event_time;
}

void LogReader::State::SeedSortedMessages() {
  if (!timestamp_mapper_) return;

  timestamp_mapper_->QueueFor(chrono::duration_cast<chrono::seconds>(
      chrono::duration<double>(FLAGS_time_estimation_buffer_seconds)));
}

void LogReader::State::Deregister() {
  for (size_t i = 0; i < channels_.size(); ++i) {
    channels_[i].reset();
  }
  remote_timestamp_senders_map_.clear();
  event_loop_unique_ptr_.reset();
  event_loop_ = nullptr;
  timer_handler_ = nullptr;
  node_event_loop_factory_ = nullptr;
}

}  // namespace logger
}  // namespace aos