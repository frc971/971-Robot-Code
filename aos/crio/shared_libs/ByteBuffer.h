#ifndef AOS_CRIO_SHARED_LIBS_BYTE_BUFFER_H_
#define AOS_CRIO_SHARED_LIBS_BYTE_BUFFER_H_

#include <algorithm>

#include "aos/common/network/ReceiveSocket.h"

namespace aos {

class ByteBuffer {
 public:
   int m_size;
   int m_length;
   int m_i;
   char *m_buffer;
   bool recv_from_sock(ReceiveSocket *sock) {
     m_length = sock->Receive(m_buffer, m_size, 40000);
     if (m_length < 0) {
       m_length = 0;
     }
     m_i = 0;
     return m_length != 0;
   }
   ByteBuffer(int size) {
     m_buffer = new char(size);
     m_size = size;
   }
   ~ByteBuffer() {
     delete m_buffer;
   }
   // Reads an uint32_t into *number and returns true on success.  *number is
   // unmodified on failure.
   bool read_uint32(uint32_t *number) {
     uint32_t vals[4];
     if (m_i + 4 > m_length) {
       m_i = m_length;
       return false;
     }
     for (int i = 0; i < 4; ++i) {
       vals[i] = read_char();
     }
     *number = vals[3] + (vals[2] << 8) + (vals[1] << 16) + (vals[0] << 24);
     return true;
   }
   float read_float() {
     if (m_i + 4 <= m_length) {
       float r;
       memcpy(&r, &m_buffer[m_i], 4);
       m_i += 4;
       return r;
     } else {
       return 1.0 / 0.0;
     }
   }
   int read_char() {
     if (m_i < m_length) {
       int val = m_buffer[m_i];
       m_i ++;
       return val;
     } else {
       return -1;
     }
   }

   int read_string(char *buf, size_t max_len) {
     int data_len = read_char();
     if (data_len <= 0) {
       return -1;
     }
     size_t to_read = std::min<size_t>(static_cast<uint8_t>(data_len), max_len);
     memcpy(buf, &m_buffer[m_i], to_read);
     m_i += to_read;
     return 0;
   }
   // Returns success or not.
   bool read_bytes(void *buf, size_t bytes) {
     if (m_length - m_i < static_cast<ssize_t>(bytes)) return false;
     memcpy(buf, &m_buffer[m_i], bytes);
     m_i += bytes;
     return true;
   }
   char *get_bytes(size_t number) {
     if (m_length - m_i < static_cast<ssize_t>(number)) return NULL;
     m_i += number;
     return &m_buffer[m_i - number];
   }
};

}  // namespace aos

#endif  // AOS_CRIO_SHARED_LIBS_BYTE_BUFFER_H_
