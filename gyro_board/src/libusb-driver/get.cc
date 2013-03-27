#include "get.h"

#include <google/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <queue>
#include <unistd.h>
#include <map>

#include "signal_handler.h"

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

bool can_frame_priority_comparator::operator() (
    const struct can_frame& x,
    const struct can_frame& y) const {
  // Error frames win first.
  if (x.can_id & CAN_EFF_FLAG) {
    return false;
  }
  if (y.can_id & CAN_EFF_FLAG) {
    return true;
  }
  // Extended frames send the first 11 bits out just like a standard frame.
  int x_standard_bits = (x.can_id & CAN_SFF_MASK);
  int y_standard_bits = (y.can_id & CAN_SFF_MASK);
  if (x_standard_bits == y_standard_bits) {
    // RTR wins next.
    bool x_rtr = (x.can_id & CAN_RTR_FLAG);
    bool y_rtr = (y.can_id & CAN_RTR_FLAG);
    if (x_rtr == y_rtr) {
      // Now check if it is an EFF packet.
      bool x_eff = (x.can_id & CAN_EFF_FLAG);
      bool y_eff = (y.can_id & CAN_EFF_FLAG);
      if (x_eff == y_eff) {
        // Now compare the bits in the extended frame.
        return (x.can_id & CAN_EFF_MASK) < (y.can_id & CAN_EFF_MASK);
      } else if (x_eff < y_eff) {
        return false;
      } else {
        return true;
      }
    } else if (x_rtr < y_rtr) {
      return true;
    } else {
      return false;
    }
  } else {
    return x_standard_bits < y_standard_bits;
  }
}

class GyroDriver::PacketReceiver : public Thread {
 public:
  // refusing to send any more frames.
  static const int kMaxRXFrames = 128;

  explicit PacketReceiver(LibUSBDeviceHandle *dev_handle)
      : Thread(), dev_handle_(dev_handle), rx_queue_(new can_priority_queue) {}

  // Serve.
  void operator()();

  bool GetCanFrame(struct can_frame *msg);

 private:
  typedef std::priority_queue<struct can_frame,
                              std::vector<struct can_frame>,
                              can_frame_priority_comparator> can_priority_queue;

  LibUSBDeviceHandle *dev_handle_;
  boost::mutex rx_mutex_;
  std::unique_ptr<can_priority_queue> rx_queue_;
  //boost::condition_variable rx_cond_;
};

GyroDriver::GyroDriver(LibUSBDeviceHandle *dev_handle)
    : dev_handle_(dev_handle), dbg_(new DbgReader(this)),
    debug_thread_(new boost::thread(boost::ref(*dbg_))),
    rx_(new GyroDriver::PacketReceiver(dev_handle)),
    rx_thread_(new boost::thread(boost::ref(*rx_))) { }


std::string GyroDriver::GetDebugData() {
  char data[64];
  int r;
  int transferred;
  r = dev_handle_->bulk_transfer(0x82,
      (unsigned char *)data, sizeof(data),
      &transferred, 0);
  return std::string(data, transferred);
}

void GyroDriver::Terminate() {
  rx_->Terminate();
}

GyroDriver::~GyroDriver() {
  rx_->Terminate();
  rx_thread_->join();
  dbg_->Terminate();
  debug_thread_->join();
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

void GyroDriver::PacketReceiver::operator()() {
  	int r;
  	int actual;
  	uint8_t data[64];
  DataStruct *real_data;
  real_data = (DataStruct *)data;
  
  while (should_run()) {
    r = dev_handle_->interrupt_transfer(
        0x81, data, sizeof(data), &actual, 1000);
    printf("size: %d\n",sizeof(DataStruct));
    CHECK_GT(actual, 0);
    
    if (r != 0) {
      LOG(ERROR) << "Read Error.  Read " << actual;
    }
    
    printf("angle: %d\n", real_data->gyro_angle);
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
  LOG(INFO) << "Receive thread down.";
}

DbgReader::DbgReader(GyroDriver *gyro)
    : Thread(), gyro_(gyro) {}

void DbgReader::operator()() {
  unsigned char data[64];
  LOG(INFO) << "Running debug dump thread.";
  while (should_run()) {
    printf("%s", gyro_->GetDebugData().c_str());
  }
  LOG(INFO) << "Exiting debug dump thread.";
}


int main(int argc, char ** argv) {
  int r;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  LibUSB libusb;

  {
    std::unique_ptr<LibUSBDeviceHandle> dev_handle(
        CHECK_NOTNULL(libusb.FindDeviceWithVIDPID(FLAGS_vid, FLAGS_pid)));

    GyroDriver gyro(dev_handle.release());

    while(true){
      	    sleep(50);
    }
  }
  return 0;
}
