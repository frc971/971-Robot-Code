#include "y2018/control_loops/superstructure/arm/graph.h"

#include <assert.h>
#include <algorithm>

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

SearchGraph::SearchGraph(size_t num_vertexes, std::initializer_list<Edge> edges)
    : SearchGraph(num_vertexes, ::std::vector<Edge>(edges)) {}

SearchGraph::SearchGraph(size_t num_vertexes, std::vector<Edge> &&edges)
    : edges_(::std::move(edges)) {
  vertexes_.resize(num_vertexes);
  size_t i = 0;
  for (const auto &edge : edges_) {
    assert(edge.start < num_vertexes);
    assert(edge.end < num_vertexes);
    assert(edge.start != edge.end);
    assert(edge.cost > 0);
    vertexes_[edge.start].forward.emplace_back(HalfEdge{edge.end, i});
    vertexes_[edge.end].reverse.emplace_back(HalfEdge{edge.start, i});
    ++i;
  }
  for (auto &vertex : vertexes_) {
    ::std::sort(vertex.forward.begin(), vertex.forward.end());
    ::std::sort(vertex.reverse.begin(), vertex.reverse.end());
  }
  vertex_heap_.Reserve(num_vertexes);
  SetGoal(0);
}

SearchGraph::~SearchGraph() {}

void SearchGraph::SetGoal(size_t vertex) {
  if (last_searched_vertex_ == vertex) {
    return;
  }
  assert(vertex < vertexes_.size());
  vertex_heap_.Clear();

  for (auto &vertex : vertexes_) {
    vertex.cached_distance = std::numeric_limits<double>::infinity();
  }

  vertexes_[vertex].cached_distance = 0.0;
  vertex_heap_.InsertOrDecrease(&vertexes_[vertex]);

  while (!vertex_heap_.Empty()) {
    Vertex *vertex = vertex_heap_.PopMin();
    for (const auto &half_edge : vertex->reverse) {
      Vertex *to_adjust = &vertexes_[half_edge.dest];
      double adjust_cost = vertex->cached_distance + GetCost(half_edge);
      if (adjust_cost < to_adjust->cached_distance) {
        to_adjust->cached_distance = adjust_cost;
        vertex_heap_.InsertOrDecrease(to_adjust);
      }
    }
  }
  last_searched_vertex_ = vertex;
}

double SearchGraph::GetCostToGoal(size_t vertex) {
  assert(vertex < vertexes_.size());
  return vertexes_[vertex].cached_distance;
}

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
