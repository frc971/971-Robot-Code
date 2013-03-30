#include "get.h"

#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>

#include <queue>
#include <iostream>
#include <map>

#include <google/gflags.h>
#include "aos/common/mutex.h"
#include "aos/common/logging/logging_impl.h"

DEFINE_int32(vid, 0x1424, "Vendor ID of the device.");
DEFINE_int32(pid, 0xd243, "Product ID of the device.");
DEFINE_bool(send, true,
            "True if the code should send packets.  (default: true)");

// USB descriptors in use:
// 0x81 Interrupt IN endpoint with the received CAN packets.
//      These packets are 64 bytes maximum, and have the same format as
//      SocketCAN.
// 0x04 Interrupt OUT endpoint with the CAN packets to send.
//      These packets are 64 bytes maximum, and have the same format as
//      SocketCAN.
// 0x82 Bulk IN endpoint with printf output.
// 0x05 Bulk OUT endpoint for stdin.

class GyroDriver::PacketReceiver : public ::aos::util::Thread {
 public:
  // refusing to send any more frames.
  static const int kMaxRXFrames = 128;

  explicit PacketReceiver(LibUSBDeviceHandle *dev_handle)
      : Thread(), dev_handle_(dev_handle) {}

  // Serve.
  virtual void Run();

  bool GetCanFrame(struct can_frame *msg);

 private:
  LibUSBDeviceHandle *dev_handle_;
  ::aos::Mutex rx_mutex_;
};

GyroDriver::GyroDriver(LibUSBDeviceHandle *dev_handle)
    : dev_handle_(dev_handle), dbg_(new DbgReader(this)),
    rx_(new GyroDriver::PacketReceiver(dev_handle)) {}


std::string GyroDriver::GetDebugData() {
  char data[64];
  int r;
  int transferred;
  r = dev_handle_->bulk_transfer(0x82,
      (unsigned char *)data, sizeof(data),
      &transferred, 0);
  return std::string(data, transferred);
}

GyroDriver::~GyroDriver() {
  rx_->Join();
  dbg_->Join();
}

//bool GyroDriver::GetCanFrame(struct can_frame *msg) {
//  return rx_->GetCanFrame(msg);
//}

struct DataStruct{
	int64_t gyro_angle;

	int32_t right_drive;
	int32_t left_drive;
	int32_t shooter_angle;
	int32_t shooter;
	int32_t indexer;
	int32_t wrist;

	int32_t capture_top_rise;
	int32_t capture_top_fall;
	int32_t capture_bottom_fall_delay;
	int32_t capture_wrist_rise;
	int32_t capture_shooter_angle_rise;

	int8_t  top_rise_count;

	int8_t top_fall_count;

	int8_t bottom_rise_count;

	int8_t bottom_fall_delay_count;
	int8_t bottom_fall_count;

	int8_t wrist_rise_count;

	int8_t shooter_angle_rise_count;
} __attribute__((__packed__));

void GyroDriver::PacketReceiver::Run() {
  int r;
  int actual;
  uint8_t data[64];
  DataStruct *real_data;
  static_assert(sizeof(*real_data) <= sizeof(data), "it doesn't fit");

  uint8_t *data_pointer = data;
  memcpy(&real_data, &data_pointer, sizeof(data_pointer));
  
  int count = 0;
  while (should_continue()) {
    r = dev_handle_->interrupt_transfer(
        0x81, data, sizeof(data), &actual, 1000);
    printf("size: %d\n",sizeof(DataStruct));
    if (actual <= 0) {
      LOG(FATAL, "didn't get any data\n");
    }
    
    if (r != 0) {
      LOG(ERROR, "Read Error. Read %d\n", actual);
    }

    ++count;
    if (count < 100) continue;
    count = 0;
    
    printf("angle: %"PRId64"\n", real_data->gyro_angle);
    printf("drivel: %d\n", real_data->left_drive);
    printf("driver: %d\n", real_data->right_drive);
    printf("shooter: %d\n", real_data->shooter);
    printf("shooter_angle: %d\n", real_data->shooter_angle);
    printf("indexer: %d\n", real_data->indexer);
    printf("wrist: %d\n", real_data->wrist);
    printf("capture:\n");
    printf("  wrist_rise{%d}: %d\n", real_data->wrist_rise_count, 
    		    real_data->capture_wrist_rise);
    printf("  shooter_angle_rise{%d}: %d\n", real_data->shooter_angle_rise_count,
    		    real_data->capture_shooter_angle_rise);
    printf("  bottom_rise_delay{%d}; \n", real_data->bottom_rise_count);
    printf("  bottom_fall_delay{%d,%d}: %d\n", real_data->bottom_fall_count,
    		    real_data->bottom_fall_delay_count,
    		    real_data->capture_bottom_fall_delay);
    printf("  top_rise{%d}: %d\n", real_data->top_rise_count,
    		    real_data->capture_top_rise);
    printf("  top_fall{%d}: %d\n", real_data->top_fall_count,
    		    real_data->capture_top_fall);




    //if (actual > 1) {
    //  rx_cond_.notify_all();
    //}
  }
  //rx_cond_.notify_all();
  LOG(INFO, "Receive thread down.");
}

DbgReader::DbgReader(GyroDriver *gyro)
    : Thread(), gyro_(gyro) {}

void DbgReader::Run() {
  LOG(INFO, "Running debug dump thread.");
  while (should_continue()) {
    printf("%s", gyro_->GetDebugData().c_str());
  }
  LOG(INFO, "Exiting debug dump thread.");
}


int main(int argc, char ** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  ::aos::logging::Init();

  LibUSB libusb;

  {
    std::unique_ptr<LibUSBDeviceHandle> dev_handle(
        libusb.FindDeviceWithVIDPID(FLAGS_vid, FLAGS_pid));
    if (!dev_handle) {
      LOG(FATAL, "couldn't find device\n");
    }

    GyroDriver gyro(dev_handle.release());

    while(true){
      	    sleep(50);
    }
  }
  return 0;
}
