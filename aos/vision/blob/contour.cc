#include "aos/vision/blob/contour.h"

namespace aos {
namespace vision {

namespace {
// Half-loop of a contour.
// This represents a range-line in a range-image
// at the current sweep of the range image propagation.
struct HalfContour {
  // start
  ContourNode *st;
  // end
  ContourNode *ed;
};

// The following helper functions handle different cases for extending
// overlapping ranges (from the first line to the next line).

// Constructs a half-contour for the first line in a range-image.
// This happens for ranges that have no overlaps in the previous line.
//   hc.st ----- hc.ed
// This is symmetric to CloseRange.
// stx is start_x; edx is end_x; alloc is the region allocator.
HalfContour FwdRange(int y, int stx, int edx, AnalysisAllocator *alloc) {
  ContourNode *st = alloc->cons_obj<ContourNode>(stx, y);
  ContourNode *ed = st;
  st->next = st;
  for (int x = stx + 1; x < edx; x++) {
    ed = alloc->cons_obj<ContourNode>(x, y, ed);
  }
  return HalfContour{st, ed};
}

// Connects hc.st to hc.ed assuming intervening range.
//   hc.st ----- hc.ed
// This closes out the contour.
void CloseRange(HalfContour hc, AnalysisAllocator *alloc) {
  auto p_end = hc.ed;
  auto p_cur = hc.st;
  while (p_cur->pt.x < p_end->pt.x - 1) {
    p_cur = p_cur->append(p_cur->pt.x + 1, p_cur->pt.y, alloc);
  }
  if (p_end->pt.x == p_cur->pt.x) {
    // Remove duplicated pixel for length 1 ranges.
    p_cur->next = p_end->next;
  } else {
    p_cur->next = p_end;
  }
}

// Connects pst to the return value (r).
// pst------
//        r------
// cst is the x position of r.
ContourNode *ExtendStart(ContourNode *pst, int cst, AnalysisAllocator *alloc) {
  while (pst->pt.x < cst) {
    pst = pst->append(pst->pt.x + 1, pst->pt.y, alloc);
  }
  pst = pst->append(pst->pt.x, pst->pt.y + 1, alloc);
  while (pst->pt.x > cst) {
    pst = pst->append(pst->pt.x - 1, pst->pt.y, alloc);
  }
  return pst;
}

// Connects pst to the return value (r)
//  ------- pst
//  --- r
// cst is the x position of r.
ContourNode *ExtendEnd(ContourNode *pst, int cst, AnalysisAllocator *alloc) {
  while (pst->pt.x > cst) {
    pst = pst->pappend(pst->pt.x - 1, pst->pt.y, alloc);
  }
  pst = pst->pappend(pst->pt.x, pst->pt.y + 1, alloc);
  while (pst->pt.x < cst) {
    pst = pst->pappend(pst->pt.x + 1, pst->pt.y, alloc);
  }
  return pst;
}

// Connects concave contour like this:
//
// --pst     est--
// ----------------
void CapRange(ContourNode *pst, ContourNode *est, AnalysisAllocator *alloc) {
  est = est->append(est->pt.x, est->pt.y + 1, alloc);
  while (est->pt.x > pst->pt.x) {
    est = est->append(est->pt.x - 1, est->pt.y, alloc);
  }
  est->next = pst;
}

// Constructs the starting range like so
// Return value (r)
//
// ----------------------
// ---r.st       r.ed----
HalfContour MakeCavity(int i, int sti, int edi, AnalysisAllocator *alloc) {
  ContourNode *st = alloc->cons_obj<ContourNode>(sti, i);
  ContourNode *ed = st;
  for (int x = sti + 1; x < edi; x++) {
    ed = ed->append(x, i, alloc);
  }
  return HalfContour{st, ed};
}
}  // namespace

// Sweepline conversion from RangeImage to ContourNode loop.
// Internal cavities are not returned.
ContourNode *RangeImgToContour(const RangeImage &rimg,
                               AnalysisAllocator *alloc) {
  alloc->reset();
  // prev list and current list plst is mutated to include ranges from the
  // current line
  // becoming clst.
  std::vector<HalfContour> clst;
  std::vector<HalfContour> plst;

  for (int x = 0; x < static_cast<int>(rimg.ranges()[0].size()); x++) {
    ImageRange rng = rimg.ranges()[0][x];
    plst.emplace_back(FwdRange(rimg.min_y(), rng.st, rng.ed, alloc));
  }

  for (int i = 1; i < static_cast<int>(rimg.size()); i++) {
    const std::vector<ImageRange> &pranges = rimg.ranges()[i - 1];
    const std::vector<ImageRange> &cranges = rimg.ranges()[i];
    int y = i + rimg.min_y();
    clst.clear();
    // prev merge id and current merge id.
    int mp = 0;
    int mc = 0;

    // This is a merge-sort of the previous list of ranges and the new-list
    // of ranges. The HafContour list above have a one to one mapping with
    // the range images here.
    while (mp < static_cast<int>(pranges.size()) &&
           mc < static_cast<int>(cranges.size())) {
      ImageRange rprev = pranges[mp];
      ImageRange rcur = cranges[mc];
      if (rcur.last() < rprev.st) {
        clst.emplace_back(FwdRange(y, rcur.st, rcur.ed, alloc));
        mc++;
      } else if (rprev.last() < rcur.st) {
        CloseRange(plst[mp], alloc);
        mp++;
      } else {
        ContourNode *within_pb = plst[mp].ed;
        ContourNode *within_ca = ExtendStart(plst[mp].st, rcur.st, alloc);

        while (true) {
          if (mp + 1 < static_cast<int>(pranges.size()) &&
              rcur.last() >= pranges[mp + 1].st) {
            mp++;
            CapRange(within_pb, plst[mp].st, alloc);
            within_pb = plst[mp].ed;
            rprev = pranges[mp];
          } else if (mc + 1 < static_cast<int>(cranges.size()) &&
                     rprev.last() >= cranges[mc + 1].st) {
            auto cav_t = MakeCavity(y, rcur.last(), cranges[mc + 1].st, alloc);
            clst.emplace_back(HalfContour{within_ca, cav_t.st});
            within_ca = cav_t.ed;
            mc++;
            rcur = cranges[mc];
          } else {
            within_pb = ExtendEnd(within_pb, rcur.last(), alloc);
            clst.emplace_back(HalfContour{within_ca, within_pb});
            mc++;
            mp++;
            break;
          }
        }
      }
    }
    while (mc < static_cast<int>(cranges.size())) {
      ImageRange rcur = cranges[mc];
      clst.emplace_back(FwdRange(y, rcur.st, rcur.ed, alloc));
      mc++;
    }

    while (mp < static_cast<int>(pranges.size())) {
      CloseRange(plst[mp], alloc);
      mp++;
    }
    std::swap(clst, plst);
  }

  for (int mp = 0; mp < static_cast<int>(plst.size()); mp++) {
    CloseRange(plst[mp], alloc);
  }
  return plst[0].st;
}

}  // namespace vision
}  // namespace aos
