#ifndef _AOS_VISION_BLOB_CODEC_H_
#define _AOS_VISION_BLOB_CODEC_H_

#include <string>

#include "aos/vision/blob/range_image.h"

namespace aos {
namespace vision {

template <typename T>
struct IntCodec {
  static constexpr size_t kSize = sizeof(T);
  static inline char *Write(char *data, T ival) {
    memcpy(data, &ival, sizeof(T));
    return data + kSize;
  }
  static inline T Read(const char **data) {
    T datum;
    memcpy(&datum, *data, sizeof(T));
    *data += kSize;
    return datum;
  }
};

using Int64Codec = IntCodec<uint64_t>;
using Int32Codec = IntCodec<uint32_t>;
using Int16Codec = IntCodec<uint16_t>;

// Calculates bytes size of blob_list. This runs the encoding algorithm below
// to get the exact size that SerializeBlob will require.
// Consider just using SerializeBlobTo instead of these lower level routines.
size_t CalculateSize(const BlobList &blob_list);
// Serializes blob_list to data. Must be valid memory of size returned by
// PredictSize.
void SerializeBlob(const BlobList &blob_list, char *data);

// Combines above to serialize to a string.
inline void SerializeBlobTo(const BlobList &blob_list, std::string *out) {
  size_t len = CalculateSize(blob_list);
  out->resize(len, 0);
  SerializeBlob(blob_list, &(*out)[0]);
}

// Parses a blob from data (Advancing data pointer by the size of the image).
const char *ParseBlobList(BlobList *blob_list, const char *data);

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_BLOB_CODEC_H_
