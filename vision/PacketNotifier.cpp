#include "vision/PacketNotifier.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>

namespace frc971 {
namespace vision {
	
void PacketNotifier::RegisterSender(){
	close(fd[1]);
}
void PacketNotifier::RegisterReciever(){
	close(fd[0]);
}
PacketNotifier *PacketNotifier::MMap(size_t data_size){
	PacketNotifier *data;
	data = (PacketNotifier *)mmap(NULL, ((data_size * 3 + 4095 + sizeof(PacketNotifier)) / 4096) * 4096, 
			PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	data->data_size = data_size;
	socketpair(PF_LOCAL, SOCK_STREAM, 0, data->fd);
	for(int i = 0;i < 3;i++){
		data->buffs[i] = (uint8_t *)data + (sizeof(PacketNotifier) + i * data_size);
	}
	void *place = &(data->mutex);
	*((int *)place) = 0; // The Mutex class needs a placement new operator. (you know, keep the masses happy);
	data->in_flight = false;
	data->to_send = -1;
	data->sending = -1;
	return data;
}
void *PacketNotifier::GetBuffer(){
	mutex.Lock();
	for(int i = 0; i < 3 ; i++){ //search for open spot.
		if(i != sending && i != to_send){ //open
			filling = i;
			mutex.Unlock();
			printf("leasing out to fill buff # %d\n",i);
			return buffs[i];
		}
	}
	mutex.Unlock();
	printf("Error in the fabric of the universe\n");
	exit(-42);
}
void PacketNotifier::Notify(){
	mutex.Lock();
	to_send = filling;
	filling = -1;
	mutex.Unlock();
	// wall error
	if(write(fd[0],"\n",1)) {}
}

void PacketNotifier::DataSent(const void * /*data*/, size_t /*datalen*/){
	printf("packet_sent: %d; fill: %d; to_send: %d \n",sending,filling,to_send);
	mutex.Lock();
	sending = -1;
	mutex.Unlock();
	in_flight = false;
}
bool PacketNotifier::GetData(char **place_to_put,size_t *length){
	if(in_flight) return false;
	mutex.Lock();
	*length = data_size;
	*place_to_put = (char *)buffs[to_send];
	sending = to_send;
	to_send = -1;
	mutex.Unlock();
	in_flight = true;
	return true;
}

}  // namespace vision
}  // namespace frc971

