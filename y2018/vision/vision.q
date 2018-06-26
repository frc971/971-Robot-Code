package y2018.vision;

message VisionStatus {
  uint32_t high_frame_count;
  uint32_t low_frame_count;
};
queue VisionStatus vision_status;
