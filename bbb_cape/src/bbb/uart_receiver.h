#ifndef FCR971_INPUT_UART_RECEIVER_H_
#define FRC971_INPUT_UART_RECEIVER_H_

#include <cstdint>

#define DATA_STRUCT_NAME DataStruct
#include <bbb_cape/src/cape/data_struct.h>
#undef DATA_STRUCT_NAME

namespace bbb {
  
  class UartReceiver {
    uint32_t baud_rate_;   
    size_t packet_size_, stuffed_size_;
    int fd_;
    uint32_t buf_used_;
    char *buf_;
  
  public:
    UartReceiver(uint32_t baud_rate);
    ~UartReceiver();
    // Opens file descriptor, etc.
    int SetUp();
    int GetPacket(DataStruct *packet);
  
  };

} //bbb

#endif
