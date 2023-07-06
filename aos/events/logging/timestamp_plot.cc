#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"

#include "aos/init.h"
#include "aos/util/file.h"
#include "frc971/analysis/in_process_plotter.h"

using frc971::analysis::Plotter;

DEFINE_bool(all, false, "If true, plot *all* the nodes at once");
DEFINE_bool(bounds, false, "If true, plot the noncausal bounds too.");
DEFINE_bool(samples, true, "If true, plot the samples too.");

DEFINE_string(offsets, "",
              "Offsets to add to the monotonic clock for each node.  Use the "
              "format of node=offset,node=offest");

// Simple C++ application to read the CSV files and use the in process plotter
// to plot them.  This smokes the pants off gnuplot in terms of interactivity.

namespace aos {

// Returns all the nodes.
std::vector<std::string> Nodes() {
  const std::string start_time_file = aos::util::ReadFileToStringOrDie(
      "/tmp/timestamp_noncausal_starttime.csv");
  std::vector<std::string_view> nodes = absl::StrSplit(start_time_file, '\n');

  std::vector<std::string> formatted_nodes;
  for (const std::string_view n : nodes) {
    if (n == "") {
      continue;
    }

    std::vector<std::string_view> l = absl::StrSplit(n, ", ");
    CHECK_EQ(l.size(), 2u) << "'" << n << "'";
    formatted_nodes.emplace_back(l[0]);
  }

  return formatted_nodes;
}

std::string SampleFile(std::string_view node1, std::string_view node2) {
  return absl::StrCat("/tmp/timestamp_noncausal_", node1, "_", node2,
                      "_samples.csv");
}

std::pair<std::vector<double>, std::vector<double>> ReadSamples(
    std::string_view node1, std::string_view node2, bool flip) {
  std::vector<double> samplefile12_t;
  std::vector<double> samplefile12_o;
  const std::string path = SampleFile(node1, node2);

  if (!aos::util::PathExists(path)) {
    return {};
  }

  const std::string file = aos::util::ReadFileToStringOrDie(path);
  bool first = true;
  std::vector<std::string_view> lines = absl::StrSplit(file, '\n');
  samplefile12_t.reserve(lines.size());
  for (const std::string_view n : lines) {
    if (first) {
      first = false;
      continue;
    }
    if (n == "") {
      continue;
    }

    std::vector<std::string_view> l = absl::StrSplit(n, ", ");
    if (l.size() != 4u) {
      continue;
    }
    double t;
    double o;
    CHECK(absl::SimpleAtod(l[0], &t));
    CHECK(absl::SimpleAtod(l[1], &o));
    samplefile12_t.emplace_back(t);
    samplefile12_o.emplace_back(flip ? -o : o);
  }
  return std::make_pair(samplefile12_t, samplefile12_o);
}

void Offset(std::vector<double> *v, double offset) {
  for (double &x : *v) {
    x += offset;
  }
}

// Returns all the nodes which talk to each other.
std::vector<std::pair<std::string, std::string>> NodeConnections() {
  const std::vector<std::string> nodes = Nodes();
  std::vector<std::pair<std::string, std::string>> result;
  for (size_t i = 1; i < nodes.size(); ++i) {
    for (size_t j = 0; j < i; ++j) {
      const std::string_view node1 = nodes[j];
      const std::string_view node2 = nodes[i];
      if (aos::util::PathExists(SampleFile(node1, node2)) ||
          aos::util::PathExists(SampleFile(node2, node1))) {
        result.emplace_back(node1, node2);
        LOG(INFO) << "Found pairing " << node1 << ", " << node2;
      }
    }
  }
  return result;
}

// Class to encapsulate the plotter state to make it easy to plot multiple
// connections.
class NodePlotter {
 public:
  NodePlotter() : nodes_(Nodes()) {
    plotter_.AddFigure("Time");
    if (!FLAGS_offsets.empty()) {
      for (std::string_view nodeoffset : absl::StrSplit(FLAGS_offsets, ',')) {
        std::vector<std::string_view> node_offset =
            absl::StrSplit(nodeoffset, '=');
        CHECK_EQ(node_offset.size(), 2u);
        double o;
        CHECK(absl::SimpleAtod(node_offset[1], &o));
        offset_.emplace(std::string(node_offset[0]), o);
      }
    }
  }

  void AddNodes(std::string_view node1, std::string_view node2);

  void Serve() {
    plotter_.Publish();
    plotter_.Spin();
  }

 private:
  std::pair<std::vector<double>, std::vector<double>> ReadLines(
      std::string_view node1, std::string_view node2, bool flip);

  std::pair<std::vector<double>, std::vector<double>> ReadOffset(
      std::string_view node1, std::string_view node2);

  double TimeOffset(std::string_view node) {
    auto it = offset_.find(std::string(node));
    if (it == offset_.end()) {
      return 0.0;
    } else {
      return it->second;
    }
  }

  std::map<std::string, double> offset_;

  Plotter plotter_;

  std::vector<std::string> nodes_;
};

std::pair<std::vector<double>, std::vector<double>> NodePlotter::ReadLines(
    std::string_view node1, std::string_view node2, bool flip) {
  std::vector<double> samplefile12_t;
  std::vector<double> samplefile12_o;
  const std::string path =
      absl::StrCat("/tmp/timestamp_noncausal_", node1, "_", node2, ".csv");

  if (!aos::util::PathExists(path)) {
    return {};
  }

  const std::string file = aos::util::ReadFileToStringOrDie(path);
  bool first = true;
  std::vector<std::string_view> lines = absl::StrSplit(file, '\n');
  samplefile12_t.reserve(lines.size());
  for (const std::string_view n : lines) {
    if (first) {
      first = false;
      continue;
    }
    if (n == "") {
      continue;
    }

    std::vector<std::string_view> l = absl::StrSplit(n, ", ");
    if (l.size() != 3u) {
      continue;
    }
    double t;
    double o;
    CHECK(absl::SimpleAtod(l[0], &t));
    CHECK(absl::SimpleAtod(l[2], &o));
    samplefile12_t.emplace_back(t);
    samplefile12_o.emplace_back(flip ? -o : o);
  }
  return std::make_pair(samplefile12_t, samplefile12_o);
}

std::pair<std::vector<double>, std::vector<double>> NodePlotter::ReadOffset(
    std::string_view node1, std::string_view node2) {
  int node1_index = -1;
  int node2_index = -1;

  {
    int index = 0;
    for (const std::string &n : nodes_) {
      if (n == node1) {
        node1_index = index;
      }
      if (n == node2) {
        node2_index = index;
      }
      ++index;
    }
  }
  CHECK_NE(node1_index, -1) << ": Unknown node " << node1;
  CHECK_NE(node2_index, -1) << ": Unknown node " << node2;
  std::vector<double> offsetfile_t;
  std::vector<double> offsetfile_o;

  const std::string file =
      aos::util::ReadFileToStringOrDie("/tmp/timestamp_noncausal_offsets.csv");
  bool first = true;
  std::vector<std::string_view> lines = absl::StrSplit(file, '\n');
  offsetfile_t.reserve(lines.size());
  for (const std::string_view n : lines) {
    if (first) {
      first = false;
      continue;
    }
    if (n == "") {
      continue;
    }

    std::vector<std::string_view> l = absl::StrSplit(n, ", ");
    CHECK_LT(static_cast<size_t>(node1_index + 1), l.size());
    CHECK_LT(static_cast<size_t>(node2_index + 1), l.size());
    double t;
    double o1;
    double o2;
    CHECK(absl::SimpleAtod(l[0], &t));
    CHECK(absl::SimpleAtod(l[1 + node1_index], &o1));
    CHECK(absl::SimpleAtod(l[1 + node2_index], &o2));
    offsetfile_t.emplace_back(t);
    offsetfile_o.emplace_back(o2 - o1);
  }
  return std::make_pair(offsetfile_t, offsetfile_o);
}

void NodePlotter::AddNodes(std::string_view node1, std::string_view node2) {
  const double offset1 = TimeOffset(node1);
  const double offset2 = TimeOffset(node2);

  std::pair<std::vector<double>, std::vector<double>> samplefile12 =
      ReadSamples(node1, node2, false);
  std::pair<std::vector<double>, std::vector<double>> samplefile21 =
      ReadSamples(node2, node1, true);

  std::pair<std::vector<double>, std::vector<double>> noncausalfile12 =
      ReadLines(node1, node2, false);
  std::pair<std::vector<double>, std::vector<double>> noncausalfile21 =
      ReadLines(node2, node1, true);

  std::pair<std::vector<double>, std::vector<double>> offsetfile =
      ReadOffset(node1, node2);

  Offset(&samplefile12.second, offset2 - offset1);
  Offset(&samplefile21.second, offset2 - offset1);
  Offset(&noncausalfile12.second, offset2 - offset1);
  Offset(&noncausalfile21.second, offset2 - offset1);
  Offset(&offsetfile.second, offset2 - offset1);

  CHECK_EQ(samplefile12.first.size(), samplefile12.second.size());
  CHECK_EQ(samplefile21.first.size(), samplefile21.second.size());
  CHECK_EQ(noncausalfile12.first.size(), noncausalfile12.second.size());
  CHECK_EQ(noncausalfile21.first.size(), noncausalfile21.second.size());

  LOG(INFO) << samplefile12.first.size() + samplefile21.first.size() +
                   noncausalfile12.first.size() + noncausalfile21.first.size()
            << " points";

  plotter_.AddLine(offsetfile.first, offsetfile.second,
                   Plotter::LineOptions{
                       .label = absl::StrCat("filter ", node2, " ", node1),
                       // TODO(austin): roboRIO compiler wants all the fields
                       // filled out, but other compilers don't...  Sigh.
                       .line_style = "*-",
                       .color = "yellow",
                       .point_size = 2.0});

  if (FLAGS_samples) {
    plotter_.AddLine(samplefile12.first, samplefile12.second,
                     Plotter::LineOptions{
                         .label = absl::StrCat("sample ", node1, " ", node2),
                         .line_style = "*",
                         .color = "purple",
                     });
    plotter_.AddLine(samplefile21.first, samplefile21.second,
                     Plotter::LineOptions{
                         .label = absl::StrCat("sample ", node2, " ", node1),
                         .line_style = "*",
                         .color = "green",
                     });
  }

  if (FLAGS_bounds) {
    plotter_.AddLine(
        noncausalfile12.first, noncausalfile12.second,
        Plotter::LineOptions{.label = absl::StrCat("nc ", node1, " ", node2),
                             .line_style = "-",
                             .color = "blue"});
    plotter_.AddLine(
        noncausalfile21.first, noncausalfile21.second,
        Plotter::LineOptions{.label = absl::StrCat("nc ", node2, " ", node1),
                             .line_style = "-",
                             .color = "orange"});
  }
}

int Main(int argc, const char *const *argv) {
  NodePlotter plotter;

  if (FLAGS_all) {
    const std::vector<std::pair<std::string, std::string>> connections =
        NodeConnections();
    for (std::pair<std::string, std::string> ab : connections) {
      plotter.AddNodes(ab.first, ab.second);
    }
    if (connections.size() == 0) {
      LOG(WARNING) << "No connections found, is something wrong?";
    }
  } else {
    CHECK_EQ(argc, 3);

    LOG(INFO) << argv[1];
    LOG(INFO) << argv[2];

    const std::string_view node1 = argv[1];
    const std::string_view node2 = argv[2];

    plotter.AddNodes(node1, node2);
  }

  plotter.Serve();

  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::Main(argc, argv);
}
