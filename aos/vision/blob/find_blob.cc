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

BlobList FindBlobs(const RangeImage &rimg) {
  BlobDisjointSet blob_set;
  // prev and current ids.
  std::vector<int> pids;
  std::vector<int> cids;
  for (ImageRange rng : rimg.ranges()[0]) {
    pids.push_back(blob_set.AddBlob(0, rng));
  }

  for (int i = 1; i < rimg.size(); i++) {
    int mi = 0;
    int mj = 0;
    const std::vector<ImageRange> &pranges = rimg.ranges()[i - 1];
    const std::vector<ImageRange> &cranges = rimg.ranges()[i];
    cids.clear();

    // Merge sort pids and cids.
    while (mi < static_cast<int>(pranges.size()) &&
           mj < static_cast<int>(cranges.size())) {
      ImageRange rprev = pranges[mi];
      ImageRange rcur = cranges[mj];
      if (rcur.last() < rprev.st) {
        if (static_cast<int>(cids.size()) == mj) {
          cids.push_back(blob_set.AddBlob(i, cranges[mj]));
        }
        mj++;
      } else if (rprev.last() < rcur.st) {
        mi++;
      } else {
        if (static_cast<int>(cids.size()) > mj) {
          blob_set.MergeInBlob(cids[mj], pids[mi]);
        } else {
          cids.push_back(blob_set.AddToBlob(pids[mi], i, cranges[mj]));
        }
        if (rcur.last() < rprev.last()) {
          mj++;
        } else {
          mi++;
        }
      }
    }
    while (mj < static_cast<int>(cranges.size())) {
      if (static_cast<int>(cids.size()) == mj) {
        cids.push_back(blob_set.AddBlob(i, cranges[mj]));
      }
      mj++;
    }
    std::swap(pids, cids);
  }
  return blob_set.MoveBlobs();
}

}  // namespace vision
}  // namespace aos
