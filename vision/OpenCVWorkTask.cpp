#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <vector>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include "opencv2/opencv.hpp"

#include "aos/common/time.h"
#include "aos/atom_code/camera/Buffers.h"
#include "aos/externals/libjpeg/include/jpeglib.h"

#include "vision/OpenCVWorkTask.h"
#include "vision/CameraProcessor.h"
#include "vision/BinaryServer.h"
#include "vision/PacketNotifier.h"
#include "vision/JPEGRoutines.h"



namespace frc971 {
namespace vision {

}  // namespace vision
}  // namespace frc971

void reciever_main(frc971::vision::PacketNotifier *notify){
  frc971::vision::BinaryServer test(8020,notify);
}

#include "frc971/queues/CameraTarget.q.h"
using frc971::vision::targets;

#include <iostream>
void sender_main(frc971::vision::PacketNotifier *notify){
  ::aos::InitNRT();
  ::aos::camera::Buffers buffers;
  CvSize size;
  size.width = ::aos::camera::Buffers::Buffers::kWidth;
  size.height = ::aos::camera::Buffers::Buffers::kHeight;

  // create processing object
  ProcessorData processor(size.width, size.height, false);

  printf("started sender main\n");
  while(true){
    //usleep(7500);
    size_t data_size;
  	timeval timestamp_timeval;
    const void *image = buffers.GetNext(
			true, &data_size, &timestamp_timeval, NULL);
	::aos::time::Time timestamp(timestamp_timeval);

    //TODO(pschuh): find proper way to cast away const for this: :(
	// parker you prolly want const_cast<type>(var);
    printf("\nNew Image Recieved\n");
    std::cout << timestamp << std::endl; 
    unsigned char *buffer = (unsigned char *)notify->GetBuffer();
	frc971::vision::process_jpeg(buffer, (unsigned char *)image, data_size);
    // build the headers on top of the buffer
	cvSetData(processor.src_header_image,
			buffer,
			::aos::camera::Buffers::Buffers::kWidth * 3);

	// transform the buffer into targets
	processor.threshold(buffer);
	processor.getContours();
	processor.filterToTargets();

	// could be used for debug ie drawContours
	//std::vector<std::vector<cv::Point> > target_contours;
	//std::vector<std::vector<cv::Point> > best_contours;
	std::vector<Target>::iterator target_it;
	Target *best_target = NULL;
	// run through the targets
	for(target_it = processor.target_list.begin();
			target_it != processor.target_list.end(); target_it++){
		//target_contours.push_back(*(target_it->this_contour));
		// select the highest target
		if (best_target == NULL) {
			best_target = &*target_it;
		} else {
			if (target_it->height < best_target->height) {
				best_target = &*target_it;
			}
		}
	}
	// if we found one then send it on
	if (best_target != NULL) {
        targets.MakeWithBuilder()
          .percent_azimuth_off_center(
		  		  best_target->rect.centroid.y / (double)::aos::camera::Buffers::kHeight - 0.5)
          .percent_elevation_off_center(
		  		  best_target->rect.centroid.x / (double)::aos::camera::Buffers::kWidth - 0.5)
          .timestamp(timestamp.ToNSec())
          .Send();
    }
    notify->Notify();
  }
  ::aos::Cleanup();
}


int main(int /*argc*/, char** /*argv*/){
  frc971::vision::PacketNotifier *notify = frc971::vision::PacketNotifier::MMap(
      ::aos::camera::Buffers::kHeight * ::aos::camera::Buffers::kWidth * 3);
  pid_t pid = fork();
  if(pid < 0){
    fprintf(stderr,"%s:%d: Error in fork()\n",__FILE__,__LINE__);
  }
  if(pid == 0){
    notify->RegisterSender();
    sender_main(notify);
  }else{
    notify->RegisterReciever();
    reciever_main(notify);
  }
}

