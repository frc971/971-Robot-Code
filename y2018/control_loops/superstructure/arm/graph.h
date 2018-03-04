#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_GRAPH_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_GRAPH_H_

#include <assert.h>
#include <stdlib.h>
#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

// Grr... normal priority queues don't allow modifying the node cost.
// This relies on pointer stability for the passed in T.
template <typename T>
class IntrusivePriorityQueue {
 public:
  // T is expected to be like so:
  //
  // struct T {
  //   IntrusivePriorityQueue<T>::QueueEntry* entry_ = nullptr;
  // };
  //
  // This QueueEntry is like a unique_ptr that always maintains a link
  // back to the unique_ptr pointing to that entry. This maintains the
  // mapping between the elements and their indexes in the list.
  // This mapping is crucial for supporting O(log(n)) decrease-key
  // operations.
  class QueueEntry {
   public:
    QueueEntry(T *value) : value_(value) { value_->entry_ = this; }

    QueueEntry(QueueEntry &&o) : value_(::std::move(o.value_)) {
      if (value_) {
        value_->entry_ = this;
      }
      o.value_ = nullptr;
    }

    QueueEntry &operator=(QueueEntry &&o) {
      if (this == &o) return *this;
      std::swap(value_, o.value_);
      if (value_) value_->entry_ = this;
      if (o.value_) o.value_->entry_ = this;
      return *this;
    }

    ~QueueEntry() {
      if (value_) value_->entry_ = nullptr;
    }

    T *get() { return value_; }

    bool operator<(const QueueEntry &o) { return *value_ < *o.value_; }

   private:
    friend class IntrusivePriorityQueue;
    T *value_ = nullptr;
  };

  // Conceptually, nodes start at inifinity and can only decrease in cost.
  // The infinity nodes are not in the queue.
  void InsertOrDecrease(T *t) {
    if (t->entry_ != nullptr) {
      std::push_heap(queue_.begin(), queue_.begin() + (GetIndex(t) + 1));
    } else {
      queue_.emplace_back(t);
      std::push_heap(queue_.begin(), queue_.end());
    }
  }

  // For testing.
  void VerifyCorrectness() {
    for (size_t i = 0; i < queue_.size(); ++i) {
      assert(queue_[i].value_->entry_ == &queue_[i]);
    }

    assert(std::is_heap(queue_.begin(), queue_.end()));
  }

  T *PopMin() {
    T *result = queue_.front().get();
    std::pop_heap(queue_.begin(), queue_.end());
    queue_.pop_back();
    return result;
  }

  bool Empty() const { return queue_.empty(); }

  void Clear() { queue_.clear(); }

  void Reserve(size_t n) { queue_.reserve(n); }

 private:
  size_t GetIndex(T *t) { return t->entry_ - &queue_[0]; }

  std::vector<QueueEntry> queue_;
};

// You pass in a graph (with num_vertexes) and a set of edges described by
// edges.
// Then, this will search that graph to find the optimal path from all points
// to a goal point.
//
// A traversal algorithm would be as such:
// // Initialize graph...
// graph.SetGoal(my_goal);
// size_t current_node = node;
//
// while (current_node != my_goal) {
//   edge = argmin(GetCostToGoal(edge.dest) for edge in
//   Neighbors(current_node));
//   Travel along edge # edge.edge_id.
//   current_node = edge.dest;
// }
class SearchGraph {
 public:
  struct Edge {
    size_t start;
    size_t end;
    double cost;
  };

  // The part of an edge stored at every vertex.
  struct HalfEdge {
    size_t dest;
    // This id matches the corresponding id in the edges array from the
    // constructor.
    size_t edge_id;
    bool operator<(const HalfEdge &o) const { return dest < o.dest; }
  };

  SearchGraph(size_t num_vertexes, std::vector<Edge> &&edges);
  SearchGraph(size_t num_vertexes, ::std::initializer_list<Edge> edges);
  SearchGraph(SearchGraph &&o)
      : vertexes_(::std::move(o.vertexes_)),
        edges_(::std::move(o.edges_)),
        vertex_heap_(::std::move(o.vertex_heap_)) {
    last_searched_vertex_ = std::numeric_limits<size_t>::max();
  }

  ~SearchGraph();

  // Set the goal vertex.
  void SetGoal(size_t vertex);

  // Compute the cost if you're at vertex.
  double GetCostToGoal(size_t vertex);

  // For walking the graph yourself.
  const std::vector<HalfEdge> &Neighbors(size_t vertex) {
    assert(vertex < vertexes_.size());
    return vertexes_[vertex].forward;
  }

  size_t num_vertexes() const { return vertexes_.size(); }

  const std::vector<Edge> &edges() const { return edges_; }

 private:
  struct Vertex {
    std::vector<HalfEdge> forward;
    std::vector<HalfEdge> reverse;
    double cached_distance = std::numeric_limits<double>::infinity();

    bool operator<(const Vertex &o) {
      return cached_distance > o.cached_distance;
    }

    IntrusivePriorityQueue<Vertex>::QueueEntry *entry_ = nullptr;
  };

  double GetCost(const HalfEdge &edge) { return edges_[edge.edge_id].cost; }
  std::vector<Vertex> vertexes_;
  std::vector<Edge> edges_;

  IntrusivePriorityQueue<Vertex> vertex_heap_;

  size_t last_searched_vertex_ = std::numeric_limits<size_t>::max();
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_GRAPH_H_
