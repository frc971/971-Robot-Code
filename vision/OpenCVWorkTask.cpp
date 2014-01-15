#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include <vector>
#include <iostream>

#include "libjpeg/include/jpeglib.h"

#include "aos/common/time.h"
#include "aos/linux_code/camera/Buffers.h"
#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"

#include "vision/OpenCVWorkTask.h"
#include "vision/CameraProcessor.h"
#include "vision/JPEGRoutines.h"


namespace frc971 {
namespace vision {

}  // namespace vision
}  // namespace frc971

namespace {
void SaveImageToFile(IplImage *image, const char *filename) {
  FILE *file = fopen(filename, "w");

  fputs("P3\n320 240\n255\n", file);
  ::cv::Mat img(image);
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      // You can now access the pixel value with cv::Vec3b
      fprintf(file, "%d %d %d ",
          img.at<cv::Vec3b>(i,j)[0],
          img.at<cv::Vec3b>(i,j)[1],
          img.at<cv::Vec3b>(i,j)[2]);

    }
    fputs("\n", file);
  }
  fclose(file);
}
}

#include "frc971/queues/CameraTarget.q.h"
using frc971::vision::targets;

void sender_main(){
  ::aos::camera::Buffers buffers;
  CvSize size;
  size.width = ::aos::camera::Buffers::Buffers::kWidth;
  size.height = ::aos::camera::Buffers::Buffers::kHeight;
  unsigned char buffer[::aos::camera::Buffers::Buffers::kWidth *
      ::aos::camera::Buffers::Buffers::kHeight * 3];

  // create processing object
  ProcessorData processor(size.width, size.height, false);

  printf("started sender main\n");
  LOG(INFO, "Camera server started\n");
  while(true){
    //usleep(7500);
    size_t data_size;
  	timeval timestamp_timeval;
    LOG(DEBUG, "getting new image\n");
    const void *image = buffers.GetNext(
		    true, &data_size, &timestamp_timeval, NULL);
    ::aos::time::Time timestamp(timestamp_timeval);

    LOG(DEBUG, "Got new image\n");
    frc971::vision::process_jpeg(
        buffer, static_cast<unsigned char *>(const_cast<void *>(image)),
        data_size);

    // build the headers on top of the buffer
    cvSetData(processor.src_header_image,
              buffer,
              ::aos::camera::Buffers::Buffers::kWidth * 3);

    // Reset.
    processor.clear();
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
    LOG(DEBUG, "Found %u targets\n", processor.target_list.size());
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
      LOG(DEBUG, "Target is %f\n", best_target->rect.centroid.x);
      targets.MakeWithBuilder()
        .percent_azimuth_off_center(
            best_target->rect.centroid.y / (double)::aos::camera::Buffers::kHeight - 0.5)
        .percent_elevation_off_center(
            best_target->rect.centroid.x / (double)::aos::camera::Buffers::kWidth - 0.5)
        .timestamp(timestamp.ToNSec())
        .Send();
    }
    //static int counter = 0;
    //if (++counter > 2) {
      //break;
    //}
  }
}


int main(int /*argc*/, char** /*argv*/){
  ::aos::InitNRT();
  sender_main();
  ::aos::Cleanup();
}

