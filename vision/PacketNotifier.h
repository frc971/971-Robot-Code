#ifndef FRC971_VISION_PACKET_NOTIFIER_H_
#define FRC971_VISION_PACKET_NOTIFIER_H_
#include "event2/buffer.h"
#include "event2/event.h"
#include "event2/listener.h"
#include "event2/bufferevent.h"
#include "aos/common/mutex.h"

#include <sys/types.h> 
#include <sys/socket.h>

namespace frc971 {
namespace vision {

/* This class lives in shared memory (using an anonomous mmap) to transfer data between
 * the server process and the image processing process.
 */
struct PacketNotifier{
 aos::Mutex mutex;
 int fd[2];
 //3 things can be happening:
 //something waiting to be sent, something sending, and something getting filled (decompressed to)
 void *buffs[3];
 int to_send;
 int filling;
 int sending;
 bool in_flight;
 size_t data_size;
 void Notify();
 void RegisterSender();
 void RegisterReciever();
 int RecieverFD(){ return fd[1]; }
 static PacketNotifier *MMap(size_t data_size);
 void DataSent(const void * /*data*/, size_t /*datalen*/);
 void *GetBuffer();
 static void StaticDataSent(const void *data, size_t datalen, void *self){
	 ((PacketNotifier *)(self))->DataSent(data,datalen);
 }
 bool GetData(char **place_to_put,size_t *length);
};
}  // namespace vision
}  // namespace frc971


#endif  //FRC971_VISION_PACKET_NOTIFIER_H_
