#ifndef Y2023_LOCALIZER_MAP_EXPANDER_LIB_H_
#define Y2023_LOCALIZER_MAP_EXPANDER_LIB_H_

#include <Eigen/Dense>
#include <map>

#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "frc971/vision/target_map_generated.h"
#include "y2023/localizer/relative_scoring_map_generated.h"
#include "y2023/localizer/scoring_map_generated.h"

namespace y2023::localizer {

// This function takes a RelativeScoringMap plus a TargetMap with april tag
// locations and uses it to generate a map of the various scoring locations on
// the field.
// This allows us to use a condensed JSON file (the RelativeScoringMap) to
// generate the more verbose ScoringMap, making it so that humans don't have to
// modify every single location in the verbose ScoringMap should something
// change. However, we do still want to have the full ScoringMap available
// for modifications should we discover that, e.g., an individual field has
// one individual pole or april tag or something out of aligment.
aos::FlatbufferDetachedBuffer<ScoringMap> ExpandMap(
    const RelativeScoringMap *relative_map,
    const frc971::vision::TargetMap *target_map);

inline aos::FlatbufferDetachedBuffer<ScoringMap> ExpandMap(
    std::string_view relative_map_json, std::string_view target_map_json) {
  aos::FlatbufferDetachedBuffer<RelativeScoringMap> relative =
      aos::JsonToFlatbuffer<RelativeScoringMap>(relative_map_json);
  aos::FlatbufferDetachedBuffer<frc971::vision::TargetMap> target_map =
      aos::JsonToFlatbuffer<frc971::vision::TargetMap>(target_map_json);
  return ExpandMap(&relative.message(), &target_map.message());
}

aos::FlatbufferDetachedBuffer<ScoringGrid> RelativeGridForTag(
    const ScoringGrid *absolute_grid,
    const frc971::vision::TargetMap *target_map, uint64_t tag);

aos::FlatbufferDetachedBuffer<DoubleSubstation> RelativeSubstationForTag(
    const DoubleSubstation *absolute_substation,
    const frc971::vision::TargetMap *target_map, uint64_t tag);

std::map<uint64_t, Eigen::Affine3d> AprilTagPoses(
    const frc971::vision::TargetMap *map);

}  // namespace y2023::localizer
#endif  // Y2023_LOCALIZER_MAP_EXPANDER_LIB_H_
