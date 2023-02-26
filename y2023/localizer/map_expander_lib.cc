#include "y2023/localizer/map_expander_lib.h"

#include "y2023/localizer/utils.h"

namespace y2023::localizer {
namespace {
flatbuffers::Offset<frc971::vision::Position> AbsolutePositionForTagAndRelative(
    const Eigen::Affine3d &tag, const frc971::vision::Position *relative,
    flatbuffers::FlatBufferBuilder *fbb) {
  const Eigen::Vector3d absolute =
      tag * Eigen::Vector3d(relative->x(), relative->y(), relative->z());
  return frc971::vision::CreatePosition(*fbb, absolute.x(), absolute.y(),
                                        absolute.z());
}

flatbuffers::Offset<ScoringRow> AbsoluteRowForTagAndRelative(
    const Eigen::Affine3d &tag, const ScoringRow *relative_row,
    flatbuffers::FlatBufferBuilder *fbb) {
  auto left_offset =
      AbsolutePositionForTagAndRelative(tag, relative_row->left_cone(), fbb);
  auto cube_offset =
      AbsolutePositionForTagAndRelative(tag, relative_row->cube(), fbb);
  auto right_offset =
      AbsolutePositionForTagAndRelative(tag, relative_row->right_cone(), fbb);
  return CreateScoringRow(*fbb, left_offset, cube_offset, right_offset);
}

flatbuffers::Offset<ScoringGrid> AbsoluteGridForTagAndRelative(
    const Eigen::Affine3d &tag, const ScoringGrid *relative_grid,
    flatbuffers::FlatBufferBuilder *fbb) {
  auto bottom_offset =
      AbsoluteRowForTagAndRelative(tag, relative_grid->bottom(), fbb);
  auto middle_offset =
      AbsoluteRowForTagAndRelative(tag, relative_grid->middle(), fbb);
  auto top_offset =
      AbsoluteRowForTagAndRelative(tag, relative_grid->top(), fbb);
  return CreateScoringGrid(*fbb, bottom_offset, middle_offset, top_offset);
}

flatbuffers::Offset<ScoringGrid> RelativeGridForTagAndAbsolute(
    const Eigen::Affine3d &tag, const ScoringGrid *absolute_grid,
    flatbuffers::FlatBufferBuilder *fbb) {
  return AbsoluteGridForTagAndRelative(tag.inverse(), absolute_grid, fbb);
}

flatbuffers::Offset<DoubleSubstation> AbsoluteSubstationForTagAndRelative(
    const Eigen::Affine3d &tag, const DoubleSubstation *relative_station,
    flatbuffers::FlatBufferBuilder *fbb) {
  auto left_offset =
      AbsolutePositionForTagAndRelative(tag, relative_station->left(), fbb);
  auto right_offset =
      AbsolutePositionForTagAndRelative(tag, relative_station->right(), fbb);
  return CreateDoubleSubstation(*fbb, left_offset, right_offset);
}

flatbuffers::Offset<DoubleSubstation> RelativeSubstationForTagAndAbsolute(
    const Eigen::Affine3d &tag, const DoubleSubstation *absolute_station,
    flatbuffers::FlatBufferBuilder *fbb) {
  return AbsoluteSubstationForTagAndRelative(tag.inverse(), absolute_station,
                                             fbb);
}

flatbuffers::Offset<HalfField> AbsoluteHalfForTagsAndRelative(
    const std::map<uint64_t, Eigen::Affine3d> &april_tags,
    const RelativeHalfField *relative_half, const RelativeScoringMap *map,
    flatbuffers::FlatBufferBuilder *fbb) {
  auto substation_offset = AbsoluteSubstationForTagAndRelative(
      april_tags.at(relative_half->substation()), map->substation(), fbb);
  auto left_offset = AbsoluteGridForTagAndRelative(
      april_tags.at(relative_half->left()), map->nominal_grid(), fbb);
  auto middle_offset = AbsoluteGridForTagAndRelative(
      april_tags.at(relative_half->middle()), map->nominal_grid(), fbb);
  auto right_offset = AbsoluteGridForTagAndRelative(
      april_tags.at(relative_half->right()), map->nominal_grid(), fbb);
  return CreateHalfField(*fbb, substation_offset, left_offset, middle_offset,
                         right_offset);
}
}  // namespace

std::map<uint64_t, Eigen::Affine3d> AprilTagPoses(
    const frc971::vision::TargetMap *map) {
  std::map<uint64_t, Eigen::Affine3d> april_tags;
  for (const frc971::vision::TargetPoseFbs *target : *map->target_poses()) {
    const uint64_t id = target->id();
    CHECK(april_tags.count(id) == 0);
    april_tags[id] = PoseToTransform(target);
  }
  return april_tags;
}

aos::FlatbufferDetachedBuffer<ScoringMap> ExpandMap(
    const RelativeScoringMap *relative_map,
    const frc971::vision::TargetMap *target_map) {
  std::map<uint64_t, Eigen::Affine3d> april_tags = AprilTagPoses(target_map);
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreateScoringMap(
      fbb,
      AbsoluteHalfForTagsAndRelative(april_tags, relative_map->red(),
                                     relative_map, &fbb),
      AbsoluteHalfForTagsAndRelative(april_tags, relative_map->blue(),
                                     relative_map, &fbb)));
  return fbb.Release();
}

aos::FlatbufferDetachedBuffer<ScoringGrid> RelativeGridForTag(
    const ScoringGrid *absolute_grid,
    const frc971::vision::TargetMap *target_map, uint64_t tag) {
  const Eigen::Affine3d tag_pose = AprilTagPoses(target_map).at(tag);
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(RelativeGridForTagAndAbsolute(tag_pose, absolute_grid, &fbb));
  return fbb.Release();
}

aos::FlatbufferDetachedBuffer<DoubleSubstation> RelativeSubstationForTag(
    const DoubleSubstation *absolute_substation,
    const frc971::vision::TargetMap *target_map, uint64_t tag) {
  const Eigen::Affine3d tag_pose = AprilTagPoses(target_map).at(tag);
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(
      RelativeSubstationForTagAndAbsolute(tag_pose, absolute_substation, &fbb));
  return fbb.Release();
}

}  // namespace y2023::localizer
