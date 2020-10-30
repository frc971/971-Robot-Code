#include <iostream>
#include <map>

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"

DEFINE_bool(all, false,
            "If true, print out the channels for all nodes in the config file, "
            "not just the channels which are visible on this node.");
DEFINE_string(config, "./config.json", "File path of aos configuration");
DEFINE_bool(short_types, true,
            "Whether to show a shortened version of the type name");

int main(int argc, char **argv) {
  gflags::SetUsageMessage(
      "\nCreates graph of nodes and message channels based on the robot config "
      "file.  \n\n"
      "To save to file, run as: \n"
      "\t aos_graph_nodes > /tmp/graph.dot\n\n"
      "To display graph, run as: \n"
      "\t aos_graph_nodes | dot -Tx11");
  aos::InitGoogle(&argc, &argv);

  // Cycle through this list of colors-- here's some defaults.
  // Can use color names or Hex values of RGB, e.g., red = "#FF0000"
  std::vector<std::string> color_list = {"red",    "blue",  "orange", "green",
                                         "violet", "gold3", "magenta"};
  int color_index = 0;

  std::string channel_name;
  std::string message_type;

  // Map from source nodes to color for them
  std::map<std::string, std::string> color_map;

  if (argc > 1) {
    LOG(ERROR) << "ERROR: Got unexpected arguments\n\n";
    gflags::ShowUsageWithFlagsRestrict(argv[0], "aos/aos_graph_nodes.cc");
    return -1;
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  const aos::Configuration *config_msg = &config.message();
  aos::ShmEventLoop event_loop(config_msg);
  event_loop.SkipTimingReport();
  event_loop.SkipAosLog();

  // Open output file and print header
  std::stringstream graph_out;
  graph_out << "digraph g {" << std::endl;

  for (const aos::Channel *channel : *config_msg->channels()) {
    VLOG(1) << "Found channel " << channel->type()->string_view();
    if (FLAGS_all || aos::configuration::ChannelIsReadableOnNode(
                         channel, event_loop.node())) {
      flatbuffers::string_view type_name = channel->type()->string_view();
      if (FLAGS_short_types) {
        // Strip down to just the top level of the message type
        type_name = channel->type()->string_view().substr(
            channel->type()->string_view().rfind(".") + 1, std::string::npos);
      }

      VLOG(1) << "Found: " << channel->name()->string_view() << ' '
              << channel->type()->string_view();

      CHECK(channel->has_source_node())
          << ": Could not find source node for channel "
          << channel->type()->string_view();
      std::string source_node_name = channel->source_node()->c_str();
      VLOG(1) << "Source node name:" << channel->source_node()->c_str();

      // If we haven't seen this node yet, add to our list, with new color
      if (color_map.count(source_node_name) == 0) {
        color_map[source_node_name] = color_list[color_index];
        color_index = (color_index + 1) % color_list.size();
      }

      if (channel->has_destination_nodes()) {
        for (const aos::Connection *connection :
             *channel->destination_nodes()) {
          VLOG(1) << "Destination Node: " << connection->name()->string_view();
          graph_out << "\t" << source_node_name << " -> "
                    << connection->name()->c_str() << " [label=\""
                    << channel->name()->c_str() << "\\n"
                    << type_name << "\" color=\"" << color_map[source_node_name]
                    << "\"];" << std::endl;
        }
      }
    }
  }

  // Write out all the nodes at the end, with their respective colors
  for (const auto node_color : color_map) {
    graph_out << "\t" << node_color.first << " [color=\"" << node_color.second
              << "\"];" << std::endl;
  }

  // Close out the file
  graph_out << "}" << std::endl;

  std::cout << graph_out.str();
  return 0;
}
