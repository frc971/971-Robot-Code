#ifndef Y2019_JEVOIS_STRUCTURES_H_
#define Y2019_JEVOIS_STRUCTURES_H_

#include <stdint.h>

#include <array>
#include <bitset>
#include <chrono>

#include "Eigen/Dense"

#include "aos/containers/sized_array.h"
#include "aos/time/time.h"
#include "third_party/GSL/include/gsl/gsl"

namespace frc971 {
namespace jevois {

// The overall flow to get data to the roboRIO consists of:
//  1.  Camera captures a frame and grabs an absolute timestamp.
//  2.  Camera processes the frame.
//  3.  Camera grabs another absolute timestamp and subtracts to get a
//      camera_duration.
//  4.  Camera sends the frame via UART to the Teensy.
//  5.  Teensy grabs an absolute timestamp for the first character received.
//  6.  Teensy buffers at most one frame from each camera.
//  7.  roboRIO toggles the CS line.
//  8.  Teensy grabs an absolute timestamp for CS being asserted.
//  9.  Teensy pulls together up to three frames and adds the time each one
//      spent in its queue to the timestamps, and queues them in its SPI
//      peripheral. This all happens before the roboRIO has enough time to start
//      actually moving data.
//  10. roboRIO transfers the frames, and sends back light/status commands.

using camera_duration = std::chrono::duration<uint8_t, std::milli>;

// This file declares the shared datastructures for the JeVois-based image
// system.
//
// Note that floating-point numbers are represented with floats. This is because
// a float has more bits than we need for anything, and the Teensy can't handle
// doubles very quickly. It would probably be quick enough, but it's easier to
// just use floats and not worry about it.

struct Target {
  bool operator==(const Target &other) const {
    if (other.distance != distance) {
      return false;
    }
    if (other.height != height) {
      return false;
    }
    if (other.heading != heading) {
      return false;
    }
    if (other.skew != skew) {
      return false;
    }
    return true;
  }
  bool operator!=(const Target &other) const {
    return !(*this == other);
  }

  // Distance to the target in meters. Specifically, the distance from the
  // center of the camera's image plane to the center of the target.
  float distance;

  // Height of the target in meters. Specifically, the distance from the camera
  // to the center of the target.
  float height;

  // Heading of the center of the target in radians. Zero is straight out
  // perpendicular to the camera's image plane. Images to the left (looking at a
  // camera image) are at a positive angle.
  float heading;

  // The angle between the target and the camera's image plane. This is
  // projected so both are assumed to be perpendicular to the floor. Parallel
  // targets have a skew of zero. Targets rotated such that their left edge
  // (looking at a camera image) is closer are at a positive angle.
  float skew;
};

// The information extracted from a single camera frame.
//
// This is all the information sent from each camera to the Teensy.
struct CameraFrame {
  bool operator==(const CameraFrame &other) const {
    if (other.targets != targets) {
      return false;
    }
    if (other.age != age) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraFrame &other) const {
    return !(*this == other);
  }

  // The top most interesting targets found in this frame.
  aos::SizedArray<Target, 3> targets;

  // How long ago from the current time this frame was captured.
  camera_duration age;
};

// The information extracted from a single camera frame, from a given camera.
struct RoborioFrame {
  bool operator==(const RoborioFrame &other) const {
    if (other.targets != targets) {
      return false;
    }
    if (other.age != age) {
      return false;
    }
    if (other.camera_index != camera_index) {
      return false;
    }
    return true;
  }
  bool operator!=(const RoborioFrame &other) const {
    return !(*this == other);
  }

  // The top most interesting targets found in this frame.
  aos::SizedArray<Target, 3> targets;

  // How long ago from the current time this frame was captured.
  camera_duration age;
  // Which camera this is from (which position on the robot, not a serial
  // number).
  int camera_index;
};

enum class CameraCommand : char {
  // Stay in normal mode.
  kNormal,
  // Go to camera passthrough mode.
  kCameraPassthrough,
  // Go to being a useful USB device.
  kUsb,
  // Send As, which triggers the bootstrap script to drop directly into USB
  // mode.
  kAs,
  // Log camera images
  kLog,
};

// This is all the information sent from the Teensy to each camera.
struct CameraCalibration {
  bool operator==(const CameraCalibration &other) const {
    if (other.calibration != calibration) {
      return false;
    }
    if (other.teensy_now != teensy_now) {
      return false;
    }
    if (other.realtime_now != realtime_now) {
      return false;
    }
    if (other.camera_command != camera_command) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraCalibration &other) const {
    return !(*this == other);
  }

  // The calibration matrix. This defines where the camera is pointing.
  //
  // TODO(Parker): What are the details on how this is defined?
  // [0][0]: mount_angle
  // [0][1]: focal_length
  // [0][2]: barrel_mount
  Eigen::Matrix<float, 3, 4> calibration;

  // A local timestamp from the Teensy. This starts at 0 when the Teensy is
  // powered on.
  aos::monotonic_clock::time_point teensy_now;

  // A realtime timestamp from the roboRIO. This will be min_time if the roboRIO
  // has never sent anything.
  aos::realtime_clock::time_point realtime_now;

  // What mode the camera should transition into.
  CameraCommand camera_command;
};

// This is all the information the Teensy sends to the RoboRIO.
struct TeensyToRoborio {
  bool operator==(const TeensyToRoborio &other) const {
    if (other.frames != frames) {
      return false;
    }
    return true;
  }
  bool operator!=(const TeensyToRoborio &other) const {
    return !(*this == other);
  }

  // The newest frames received from up to three cameras. These will be the
  // three earliest-received of all buffered frames.
  aos::SizedArray<RoborioFrame, 3> frames;
};

// This is all the information the RoboRIO sends to the Teensy.
struct RoborioToTeensy {
  bool operator==(const RoborioToTeensy &other) const {
    if (other.beacon_brightness != beacon_brightness) {
      return false;
    }
    if (other.light_rings != light_rings) {
      return false;
    }
    if (other.realtime_now != realtime_now) {
      return false;
    }
    if (other.camera_command != camera_command) {
      return false;
    }
    return true;
  }
  bool operator!=(const RoborioToTeensy &other) const {
    return !(*this == other);
  }

  // Brightnesses for each of the beacon light channels. 0 is off, 255 is fully
  // on.
  std::array<uint8_t, 3> beacon_brightness;

  // Whether the light ring for each camera should be on.
  std::bitset<5> light_rings;

  // The current time.
  aos::realtime_clock::time_point realtime_now;

  // A command to send to all the cameras.
  CameraCommand camera_command;
};

}  // namespace jevois
}  // namespace frc971

#endif  // Y2019_JEVOIS_STRUCTURES_H_
