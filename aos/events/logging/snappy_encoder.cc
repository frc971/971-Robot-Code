#include "aos/events/logging/snappy_encoder.h"

#include "aos/events/logging/crc32.h"
#include "external/snappy/snappy.h"

namespace aos::logger {
// Snappy file format is a series of chunks. Each chunk consists of:
// 1-byte: Format identifier.
// 3-bytes: Little-endian chunk length, not counting the four bytes of format
//          + length.
//
// The relevant chunk identifiers are 0xff (header) and 0x00 (compressed data).
// Compressed data's first four bytes are a CRC-32C that is masked by some
// magic function. Technically the size of the uncompressed data per chunk
// is supposed to be no more than 65536 bytes, but I am too lazy to enforce
// that at the moment.
// 0x01 is the type for uncompressed data, which has not been implemented here.
namespace {
const std::vector<uint8_t> kSnappyIdentifierChunk = {
    0xFF, 0x06, 0x00, 0x00, 's', 'N', 'a', 'P', 'p', 'Y'};

// Magic mask that snappy's framing format requires for some reason.
uint32_t MaskChecksum(uint32_t x) {
  return ((x >> 15) | (x << 17)) + 0xa282ead8;
}

uint32_t SnappyChecksum(uint8_t *data, size_t size) {
  return MaskChecksum(ComputeCrc32({data, size}));
}
}  // namespace

SnappyEncoder::SnappyEncoder(size_t chunk_size) : chunk_size_(chunk_size) {
  queue_.emplace_back();
  queue_.back().resize(kSnappyIdentifierChunk.size());
  std::copy(kSnappyIdentifierChunk.begin(), kSnappyIdentifierChunk.end(),
            queue_.back().begin());
  total_bytes_ += queue_.back().size();
}

void SnappyEncoder::Finish() { EncodeCurrentBuffer(); }

void SnappyEncoder::Encode(flatbuffers::DetachedBuffer &&in) {
  accumulated_checksum_ =
      AccumulateCrc32({in.data(), in.size()}, accumulated_checksum_);
  buffer_source_.AppendBuffer(std::move(in));

  if (buffer_source_.Available() >= chunk_size_) {
    EncodeCurrentBuffer();
  }
}

void SnappyEncoder::EncodeCurrentBuffer() {
  if (buffer_source_.Available() == 0) {
    return;
  }
  const uint32_t uncompressed_checksum =
      MaskChecksum(accumulated_checksum_.value());
  queue_.emplace_back();
  constexpr size_t kPrefixSize = 8;
  queue_.back().resize(kPrefixSize +
                       snappy::MaxCompressedLength(buffer_source_.Available()));
  queue_.back().data()[0] = '\x00';
  char *const compressed_start =
      reinterpret_cast<char *>(queue_.back().data()) + kPrefixSize;
  snappy::UncheckedByteArraySink snappy_sink(compressed_start);
  const size_t compressed_size =
      snappy::Compress(&buffer_source_, &snappy_sink);
  CHECK_LT(compressed_size + kPrefixSize, queue_.back().size());
  queue_.back().resize(compressed_size + kPrefixSize);
  const size_t chunk_size = compressed_size + 4;
  CHECK_LT(chunk_size, 1U << 24);
  queue_.back().data()[1] = chunk_size & 0xFF;
  queue_.back().data()[2] = (chunk_size >> 8) & 0xFF;
  queue_.back().data()[3] = (chunk_size >> 16) & 0xFF;
  queue_.back().data()[4] = uncompressed_checksum & 0xFF;
  queue_.back().data()[5] = (uncompressed_checksum >> 8) & 0xFF;
  queue_.back().data()[6] = (uncompressed_checksum >> 16) & 0xFF;
  queue_.back().data()[7] = (uncompressed_checksum >> 24) & 0xFF;

  total_bytes_ += queue_.back().size();

  accumulated_checksum_.reset();
  CHECK_EQ(0u, buffer_source_.Available());
}

void SnappyEncoder::Clear(int n) {
  CHECK_GE(n, 0);
  CHECK_LE(static_cast<size_t>(n), queue_size());
  queue_.erase(queue_.begin(), queue_.begin() + n);
}

size_t SnappyEncoder::queued_bytes() const {
  size_t bytes = 0;
  for (const auto &buffer : queue_) {
    bytes += buffer.size();
  }
  return bytes;
}

std::vector<absl::Span<const uint8_t>> SnappyEncoder::queue() const {
  std::vector<absl::Span<const uint8_t>> queue;
  queue.reserve(queue_.size());
  for (const auto &buffer : queue_) {
    queue.emplace_back(buffer.data(), buffer.size());
  }
  return queue;
}

size_t SnappyEncoder::DetachedBufferSource::Available() const {
  size_t total_available = 0;
  for (const flatbuffers::DetachedBuffer &buffer : buffers_) {
    total_available += buffer.size();
  }
  return total_available;
}

const char *SnappyEncoder::DetachedBufferSource::Peek(size_t *length) {
  *CHECK_NOTNULL(length) = buffers_[0].size() - index_into_first_buffer_;
  return reinterpret_cast<char *>(buffers_[0].data()) +
         index_into_first_buffer_;
}

void SnappyEncoder::DetachedBufferSource::Skip(size_t n) {
  if (n == 0) {
    return;
  }

  CHECK(!buffers_.empty());

  index_into_first_buffer_ += n;
  CHECK_LE(index_into_first_buffer_, buffers_[0].size())
      << ": " << n << " is too large a skip.";
  if (index_into_first_buffer_ == buffers_[0].size()) {
    buffers_.erase(buffers_.begin());
    index_into_first_buffer_ = 0;
  }
}

void SnappyEncoder::DetachedBufferSource::AppendBuffer(
    flatbuffers::DetachedBuffer &&buffer) {
  buffers_.emplace_back(std::move(buffer));
}

size_t SnappyDecoder::Read(uint8_t *begin, uint8_t *end) {
  // Decoding process:
  // We maintain an uncompressed_buffer_ of data that may have been uncompressed
  // in the past but which hasn't been read yet.
  //
  // When we run out of that buffer and need to actually read new data, we
  // need to read things from the file in one-chunk units. Snappy expects to
  // uncompress an entire chunk at once, so there isn't any value in attempting
  // to be more incremental. Each chunk will be of some variable size (specified
  // at the start of the chunk). Once we read and uncompress the chunk, we can
  // then copy it into the user's buffer until either we run out of data and
  // need to uncompress a new chunk or user's buffer runs out in which case
  // we fill up their buffer and store the extra data in uncompressed_buffer_
  // to be populated later.

  uint8_t *current_output = begin;
  if (uncompressed_buffer_.size() > 0) {
    const size_t amount_to_copy =
        std::min<size_t>(uncompressed_buffer_.size(), end - current_output);
    std::memcpy(current_output, uncompressed_buffer_.data(), amount_to_copy);
    current_output += amount_to_copy;
    uncompressed_buffer_.erase_front(amount_to_copy);
  }

  while (current_output != end) {
    uint8_t header[4];
    const size_t read_length = underlying_decoder_->Read(header, header + 4);
    if (read_length == 0) {
      break;
    }
    if (read_length != 4u) {
      LOG(WARNING) << "Logfile data is truncated.";
      break;
    }
    const uint8_t chunk_type = header[0];
    const size_t chunk_size = header[1] + (header[2] << 8) + (header[3] << 16);
    compressed_buffer_.resize(chunk_size);
    if (chunk_size !=
        underlying_decoder_->Read(compressed_buffer_.data(),
                                  compressed_buffer_.data() + chunk_size)) {
      LOG(WARNING) << "Logfile data is truncated.";
      break;
    }
    if (chunk_type == 0xFF) {
      // Start of file, don't bother
      // checking the contents.
      continue;
    } else if (chunk_type == 0x00) {
      if (compressed_buffer_.size() < 4u) {
        LOG(WARNING) << "Logfile data is truncated.";
        break;
      }
      const uint32_t checksum = compressed_buffer_.data()[0] +
                                (compressed_buffer_.data()[1] << 8) +
                                (compressed_buffer_.data()[2] << 16) +
                                (compressed_buffer_.data()[3] << 24);

      const char *input_data =
          reinterpret_cast<char *>(compressed_buffer_.data() + 4);
      const size_t input_size = compressed_buffer_.size() - 4;

      size_t uncompressed_length;
      CHECK(snappy::GetUncompressedLength(input_data, input_size,
                                          &uncompressed_length));

      // If the user's buffer can fit the entire uncompressed data, we
      // will uncompress directly into their buffer. Otherwise, we uncompress
      // into a separate buffer and then copy the correct number of bytes out
      // to fill up the user's buffer.
      // TODO(james): In the future, overload a snappy::Sink to avoid
      // unnecessary copies by extracting the initial N bytes directly into
      // the user's buffer.
      if (end > (uncompressed_length + current_output)) {
        CHECK(snappy::RawUncompress(input_data, input_size,
                                    reinterpret_cast<char *>(current_output)))
            << ": Corrupted snappy chunk.";
        CHECK_EQ(checksum, SnappyChecksum(current_output, uncompressed_length))
            << ": Checksum mismatch.";

        current_output += uncompressed_length;
      } else {
        uncompressed_buffer_.resize(uncompressed_length);
        CHECK(snappy::RawUncompress(
            input_data, input_size,
            reinterpret_cast<char *>(uncompressed_buffer_.data())))
            << ": Corrupted snappy chunk.";
        CHECK_EQ(checksum, SnappyChecksum(uncompressed_buffer_.data(),
                                          uncompressed_buffer_.size()))
            << ": Checksum mismatch.";
        std::memcpy(current_output, uncompressed_buffer_.data(),
                    end - current_output);
        uncompressed_buffer_.erase_front(end - current_output);
        current_output = end;
      }
    } else {
      // Unimplemented.
      LOG(FATAL) << "Unsupported snappy chunk type "
                 << static_cast<int>(chunk_type);
    }
  }
  total_output_ += current_output - begin;

  return current_output - begin;
}

}  // namespace aos::logger
