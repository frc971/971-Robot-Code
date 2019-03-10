#include "aos/vision/blob/find_blob.h"

#include "aos/vision/blob/disjoint_set.h"

namespace aos {
namespace vision {

struct BlobBuilder {
  BlobBuilder(int i) : min_y(i) {}
  void Add(int i, ImageRange rng) {
    while (static_cast<int>(ranges.size()) <= i - min_y) {
      ranges.emplace_back();
    }
    ranges[i - min_y].push_back(rng);
  }

  void Merge(const BlobBuilder &o) {
    // Always will be positive because of std::swap below in
    // MergeInBlob maintains y order.
    int diff = o.min_y - min_y;
    for (int j = 0; j < static_cast<int>(o.ranges.size()); j++) {
      int i = j + diff;
      while (static_cast<int>(ranges.size()) <= i) {
        ranges.emplace_back();
      }

      std::vector<ImageRange> cur = ranges[i];
      std::vector<ImageRange> prev = o.ranges[j];

      // Merge sort of cur and prev.
      ranges[i].clear();
      int a = 0;
      int b = 0;
      while (a < static_cast<int>(cur.size()) &&
             b < static_cast<int>(prev.size())) {
        if (cur[a].st < prev[b].st) {
          ranges[i].push_back(cur[a++]);
        } else {
          ranges[i].push_back(prev[b++]);
        }
      }
      while (a < static_cast<int>(cur.size())) {
        ranges[i].push_back(cur[a++]);
      }
      while (b < static_cast<int>(prev.size())) {
        ranges[i].push_back(prev[b++]);
      }
    }
  }
  std::vector<std::vector<ImageRange>> ranges;
  int min_y;
};

// Uses disjoint set class to track range images.
// Joins in the disjoint set are done at the same time as joins in the
// range image.
class BlobDisjointSet {
 public:
  int AddToBlob(int bid, int i, ImageRange img) {
    bid = disjoint_set.Find(bid);
    items[bid].Add(i, img);
    return bid;
  }
  int AddBlob(int i, ImageRange img) {
    int bid = disjoint_set.Add();
    items.emplace_back(i);
    items[bid].Add(i, img);
    return bid;
  }
  void MergeInBlob(int cbid, int pbid) {
    cbid = disjoint_set.Find(cbid);
    pbid = disjoint_set.Find(pbid);
    if (cbid != pbid) {
      if (items[cbid].min_y > items[pbid].min_y) {
        std::swap(cbid, pbid);
      }
      items[cbid].Merge(items[pbid]);
      disjoint_set.ExplicitUnion(pbid, cbid);
    }
  }
  BlobList MoveBlobs() {
    std::vector<RangeImage> blobs;
    for (int i = 0; i < static_cast<int>(items.size()); i++) {
      if (disjoint_set.IsRoot(i)) {
        blobs.emplace_back(items[i].min_y, std::move(items[i].ranges));
      }
    }
    return blobs;
  }

 private:
  DisjointSet disjoint_set;
  std::vector<BlobBuilder> items;
};

BlobList FindBlobs(const RangeImage &input_image) {
  BlobDisjointSet blob_set;
  std::vector<int> previous_ids;
  std::vector<int> current_ids;
  for (ImageRange input_range : input_image.ranges()[0]) {
    previous_ids.push_back(blob_set.AddBlob(0, input_range));
  }

  for (int input_row = 1; input_row < input_image.size(); input_row++) {
    // The index of previous_ranges we're currently considering.
    int previous_location = 0;
    // The index of current_ranges we're currently considering.
    int current_location = 0;
    const std::vector<ImageRange> &previous_ranges =
        input_image.ranges()[input_row - 1];
    const std::vector<ImageRange> &current_ranges =
        input_image.ranges()[input_row];
    current_ids.clear();

    while (previous_location < static_cast<int>(previous_ranges.size()) &&
           current_location < static_cast<int>(current_ranges.size())) {
      const ImageRange previous_range = previous_ranges[previous_location];
      const ImageRange current_range = current_ranges[current_location];
      if (current_range.last() < previous_range.st) {
        // If current_range ends before previous_range starts, then they don't
        // overlap, so we might want to add current_range to a separate blob.
        if (static_cast<int>(current_ids.size()) == current_location) {
          // We only want to add it if we haven't already added this
          // current_range to a blob.
          current_ids.push_back(blob_set.AddBlob(input_row, current_range));
        }
        current_location++;
      } else if (previous_range.last() < current_range.st) {
        // If previous_range ends before current_range starts, then they don't
        // overlap. Definitely nothing to merge before current_range in the
        // current row.
        previous_location++;
      } else {
        if (static_cast<int>(current_ids.size()) > current_location) {
          // If we've already added current_range, and they still overlap, then
          // we should merge the blobs.
          blob_set.MergeInBlob(current_ids[current_location],
                               previous_ids[previous_location]);
        } else {
          // If we haven't yet added current_range to a blob, then do that now.
          current_ids.push_back(
              blob_set.AddToBlob(previous_ids[previous_location], input_row,
                                 current_ranges[current_location]));
        }
        // Increment the furthest-left-ending range in either row. The
        // further-right-ending one might merge with the next one in the other
        // row, so we need to look at it again next iteration.
        if (current_range.last() < previous_range.last()) {
          current_location++;
        } else {
          previous_location++;
        }
      }
    }
    // Finish processing the current row. This is sometimes necessary if the
    // previous row was fully processed first.
    //
    // Note that we don't care if the previous row didn't get fully iterated
    // over.
    while (current_location < static_cast<int>(current_ranges.size())) {
      if (static_cast<int>(current_ids.size()) == current_location) {
        // We only want to add it if we haven't already added this range to a
        // blob.
        current_ids.push_back(
            blob_set.AddBlob(input_row, current_ranges[current_location]));
      }
      current_location++;
    }

    // Update previous_ids for the next iteration.
    std::swap(previous_ids, current_ids);
  }
  return blob_set.MoveBlobs();
}

}  // namespace vision
}  // namespace aos
