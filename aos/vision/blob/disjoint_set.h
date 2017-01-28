#ifndef _AOS_VISION_BLOB_DISJOINT_SET_H_
#define _AOS_VISION_BLOB_DISJOINT_SET_H_

#include <vector>

namespace aos {
namespace vision {

// Disjoint set algorithm:
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
class DisjointSet {
 public:
  DisjointSet() {}
  DisjointSet(int node_count) : nodes_(node_count, -1) {}
  // Adds another node. Not connected to anything.
  // Returns id of the additional node for tracking purposes.
  int Add() {
    nodes_.push_back(-1);
    return nodes_.size() - 1;
  }
  int &operator[](int id) { return nodes_[id]; }
  // Find returns the canonical id of the set containing id.
  // This id is an arbitrary artifact of the merge process.
  int Find(int id) {
    int chained_id = nodes_[id];
    if (chained_id <= -1) {
      return id;
    }
    return nodes_[id] = Find(chained_id);
  }
  // The blob mergeing needs to explicitly override the default union behavior.
  void ExplicitUnion(int parent_id, int child_id) {
    nodes_[parent_id] = child_id;
  }
  // Check to see if this node is the canonical id.
  bool IsRoot(int id) { return nodes_[id] < 0; }

 private:
  // Implemented over a vector.
  std::vector<int> nodes_;
};

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_BLOB_DISJOINT_SET_H_
