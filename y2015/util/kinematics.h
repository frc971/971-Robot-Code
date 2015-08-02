#ifndef Y2015_UTIL_KINEMATICS_H_
#define Y2015_UTIL_KINEMATICS_H_

#include <cmath>
#include "Eigen/Dense"
#include "y2015/constants.h"

namespace aos {
namespace util {

// A class for performing forward and inverse kinematics on the elevator-arm
// system. It can calculate where the fridge grabbers will be if the arm and
// elevator are at a given position, as well as where the arm and elevator
// should go in order to get the grabbers to a specific location.
class ElevatorArmKinematics {
 public:
  typedef enum {
    // These specify the particular region that an invalid request was in. Right
    // is toward the front of the robot, left is toward the back.

    // Request is valid.
    REGION_VALID = 0,
    // Request is farther right than the arm can extend.
    REGION_RIGHT = 1 << 0,
    // Request is towards the front of the robot but higher than we can extend
    // with the elevator and the arm.
    REGION_UPPER_RIGHT = 1 << 1,
    // We can get the x part of the request, which is towards the front of the
    // robot, but not the h part, which is too high.
    REGION_INSIDE_UPPER_RIGHT = 1 << 2,
    // We can get the x part of the request, which is towards the front of the
    // robot, but not the h part, which is too low.
    REGION_INSIDE_LOWER_RIGHT = 1 << 3,
    // Request is towards the front of the robot but lower than we can extend
    // with the elevator and the arm.
    REGION_LOWER_RIGHT = 1 << 4,
    // Request is farther left than the arm can extend.
    REGION_LEFT = 1 << 5,
    // Request is towards the back of the robot but higher than we can extend
    // with the elevator and the arm.
    REGION_UPPER_LEFT = 1 << 6,
    // We can get the x part of the request, which is towards the front of the
    // robot, but not the h part, which is too high.
    REGION_INSIDE_UPPER_LEFT = 1 << 7,
    // We can get the x part of the request, which is towards the back of the
    // robot, but not the h part, which is too low.
    REGION_INSIDE_LOWER_LEFT = 1 << 8,
    // Request is towards the back of the robot but lower than we can extend
    // with the elevator and the arm.
    REGION_LOWER_LEFT = 1 << 9,
    // Request is invalid, but don't know where it is out of range.
    REGION_UNKNOWN = 1 << 10,
  } Region;

  class KinematicResult {
   public:
    // The elevator height result from an inverse kinematic.
    double elevator_height;
    // The arm angle result from an inverse kinematic.
    double arm_angle;
    // Resulting velocity of the elevator given x,y velocities.
    double elevator_velocity;
    // Resulting velocity of the arm given x,y velocities.
    double arm_velocity;
    // The fridge height value from a forward kinematic.
    double fridge_h;
    // The fridge x value from a forward kinematic.
    double fridge_x;
    // Resulting velocity of the fridge height given arm and elevator
    // velocities.
    double fridge_h_velocity;
    // Resulting velocity of the fridge x given arm and elevator velocities.
    double fridge_x_velocity;
  };

  // If we use the default constructor we wil just always not be able to move.
  ElevatorArmKinematics()
      : length_arm_(1.0),
        elevator_max_(0.0),
        elevator_min_(0.0),
        upper_angle_limit_(0.0),
        lower_angle_limit_(0.0) {}

  ElevatorArmKinematics(double length_arm, double height_max, double height_min,
                        double angle_max, double angle_min)
      : length_arm_(length_arm),
        elevator_max_(height_max),
        elevator_min_(height_min),
        upper_angle_limit_(angle_max),
        lower_angle_limit_(angle_min),
        geometry_(frc971::constants::GetValues().clawGeometry) {}

  ~ElevatorArmKinematics() {}

  // Limit a number to the speed of light. The loops should handle this a lot
  // better than overflow.
  void LimitLightSpeed(double* num) {
    if (*num > 299792458.0) {
      *num = 299792458.0;
    }
    if (*num < -299792458.0) {
      *num = -299792458.0;
    }
    if (!::std::isfinite(*num)) {
      *num = 0.0;
    }
  }

  // Calculates the arm angle in radians and the elevator height in meters for
  // a desired Fridge grabber height and x location. x is positive going
  // toward the front of the robot.
  // h is positive going up. x=0 and h=0 is the location of the top fridge
  // grabber when the elevator is at 0 height and the arm angle is 0 (vertical).
  // Both the x and h values are given in meters.
  // Returns the region of the request.
  // Result is:
  // the angle of the arm in radians
  // the height of the elevator in meters
  // the resulting x
  // and the resulting h
  // If an impossible location is requested, the arm angle and elevator height
  // returned are the closest possible for the requested fridge grabber height.
  // If the requested height is above the max possible height, the angle
  // will be 0 and the height will be the max possible height of the elevator.
  int InverseKinematic(double request_x, double request_h,
                       double request_x_velocity, double request_y_velocity,
                       KinematicResult* result) {
    int valid_or_invalid = REGION_VALID;

    double square_arm = length_arm_ * length_arm_;
    double term = ::std::sqrt(square_arm - request_x * request_x);

    // Check to see if the x location can be satisfied.  If the requested x
    // location
    // is further out than the arm can go, it is not possible for any elevator
    // location.
    if (request_x > length_arm_) {
      result->arm_angle = -M_PI * 0.5;
      valid_or_invalid |= REGION_RIGHT;
    } else if (request_x < -length_arm_) {
      result->arm_angle = M_PI * 0.5;
      valid_or_invalid |= REGION_LEFT;
    } else {
      result->arm_angle = ::std::asin(-request_x / length_arm_);
      result->arm_velocity = (-1.0 / term) * request_x_velocity;
      LimitLightSpeed(&result->arm_velocity);
    }

    result->elevator_height =
        request_h + length_arm_ * (1.0 - ::std::cos(result->arm_angle));
    result->elevator_velocity =
        (request_x / (square_arm * term)) * request_x_velocity +
        request_y_velocity;
    LimitLightSpeed(&result->elevator_velocity);

    // Check to see if the requested elevator height is possible
    if (request_h > elevator_max_) {
      // The elevator cannot go high enough with any arm angle to satisfy this
      // request. So position the elevator at the top and the arm angle set to
      // vertical.
      result->elevator_height = elevator_max_;
      result->arm_angle = 0.0;
      if (request_x >= 0) {
        valid_or_invalid |= REGION_UPPER_RIGHT;
      } else {
        valid_or_invalid |= REGION_UPPER_LEFT;
      }
    } else if (request_h < -length_arm_ + elevator_min_) {
      // The elevator cannot go low enough with any arm angle to satisfy this
      // request. So position the elevator at the bottom and the arm angle to
      // satisfy the x request The elevator will move up as the grabber moves to
      // the center of the robot when in this part of the motion space.
      result->elevator_height = elevator_min_;
      if (request_x >= 0) {
        valid_or_invalid |= REGION_LOWER_RIGHT;
      } else {
        valid_or_invalid |= REGION_LOWER_LEFT;
      }
    } else if (result->elevator_height > elevator_max_) {
      // Impossibly high request.  So get as close to the x request with the
      // elevator at the top of its range.
      result->elevator_height = elevator_max_;
      if (request_x >= 0) {
        result->arm_angle =
            -::std::acos((length_arm_ + request_h - elevator_max_) /
                         length_arm_);
        valid_or_invalid |= REGION_INSIDE_UPPER_RIGHT;
      } else {
        result->arm_angle = ::std::acos(
            (length_arm_ + request_h - elevator_max_) / length_arm_);
        valid_or_invalid |= REGION_INSIDE_UPPER_LEFT;
      }
    } else if (result->elevator_height < elevator_min_) {
      // Impossibly low request.  So satisfy the x request with the elevator
      // at the bottom of its range.
      // The elevator will move up as the grabber moves to the center of the
      // robot
      // when in this part of the motion space.
      result->elevator_height = elevator_min_;
      if (request_x >= 0) {
        valid_or_invalid |= REGION_INSIDE_LOWER_RIGHT;
      } else {
        valid_or_invalid |= REGION_INSIDE_LOWER_LEFT;
      }
    }

    // if we are not in a valid region we will zero the velocity for now
    if (valid_or_invalid != REGION_VALID) {
      result->arm_velocity = 0.0;
      result->elevator_velocity = 0.0;
    }

    if (ForwardKinematic(result->elevator_height, result->arm_angle,
                         result->elevator_velocity, result->arm_velocity,
                         result) == REGION_UNKNOWN) {
      return REGION_UNKNOWN;
    }
    return valid_or_invalid;
  }

  // Takes an elevator height and arm angle and projects these to the resulting
  // fridge height and x offset. Returns REGION_UNKNOWN if the values are
  // outside
  // limits. This will result in the height/angle being bounded and the
  // resulting position will be returned.
  Region ForwardKinematic(double elevator_height, double arm_angle,
                          double elevator_velocity, double arm_velocity,
                          KinematicResult* result) {
    result->elevator_height = elevator_height;
    result->arm_angle = arm_angle;

    Region valid = REGION_VALID;
    if (elevator_height < elevator_min_) {
      LOG(WARNING, "elevator %.2f limited at %.2f\n", result->elevator_height,
          elevator_min_);
      result->elevator_height = elevator_min_;
      valid = REGION_UNKNOWN;
    }
    if (elevator_height > elevator_max_) {
      LOG(WARNING, "elevator %.2f limited at %.2f\n", result->elevator_height,
          elevator_max_);
      result->elevator_height = elevator_max_;
      valid = REGION_UNKNOWN;
    }
    if (arm_angle < lower_angle_limit_) {
      LOG(WARNING, "arm %.2f limited at %.2f\n", result->arm_angle,
          lower_angle_limit_);
      result->arm_angle = lower_angle_limit_;
      valid = REGION_UNKNOWN;
    }
    if (arm_angle > upper_angle_limit_) {
      result->arm_angle = upper_angle_limit_;
      LOG(WARNING, "arm %.2f limited at %.2f\n", result->arm_angle,
          upper_angle_limit_);
      valid = REGION_UNKNOWN;
    }
    // Compute the fridge grabber height and x location using the computed
    // elevator height and arm angle.
    result->fridge_h = result->elevator_height +
                       (::std::cos(result->arm_angle) - 1.0) * length_arm_;
    result->fridge_x = -::std::sin(result->arm_angle) * length_arm_;
    // velocity based on joacobian
    result->fridge_x_velocity =
        -length_arm_ * ::std::cos(result->arm_angle) * arm_velocity;
    LimitLightSpeed(&result->fridge_x_velocity);
    result->fridge_h_velocity =
        -length_arm_ * ::std::sin(result->arm_angle) * arm_velocity +
        elevator_velocity;
    LimitLightSpeed(&result->fridge_h_velocity);
    return valid;
  }

  // Same as ForwardKinematic but without any checking.
  Eigen::Vector2d ForwardKinematicNoChecking(double elevator_height,
                                             double arm_angle) {
    // Compute the fridge grabber height and x location using the computed
    // elevator height and arm angle.
    Eigen::Vector2d grabber_location;
    grabber_location.y() =
        elevator_height + (::std::cos(arm_angle) - 1.0) * length_arm_;
    grabber_location.x() = -::std::sin(arm_angle) * length_arm_;
    return grabber_location;
  }

  // 2 dimensional version of cross product
  double Cross(Eigen::Vector2d a, Eigen::Vector2d b) {
    double crossProduct = a.x() * b.y() - a.y() * b.x();
    return crossProduct;
  }

  // Tell whether or not it is safe to move the grabber to a position.
  // Returns true if the current move is safe.
  // If it returns false then a safe_claw_angle that is greater than zero is
  // acceptable otherwise if safe_claw_angle is less than zero there will be no
  // valid solution.
  bool GrabberArmIntersectionCheck(double elevator_height, double arm_angle,
                                   double claw_angle, double* safe_claw_angle) {
    Eigen::Vector2d grabber_location =
        ForwardKinematicNoChecking(elevator_height, arm_angle);
    if (grabber_location.x() < geometry_.grabber_always_safe_x_max ||
        grabber_location.y() > geometry_.grabber_always_safe_h_min) {
      *safe_claw_angle = claw_angle;
      return true;
    }
    Eigen::Vector2d grabber_bottom_end;
    Eigen::Vector2d claw_i_unit_direction(::std::cos(claw_angle),
                                          sin(claw_angle));
    Eigen::Vector2d claw_j_unit_direction(-::std::sin(claw_angle),
                                          cos(claw_angle));

    // Vector from the center of the arm rotation axis to front bottom
    // corner of the grabber.
    Eigen::Vector2d grabber_end_location_from_arm_axis(
        geometry_.grabber_half_length, -geometry_.grabber_delta_y);

    // Bottom front corner of the grabber.  This is what will usually hit the
    // claw first.
    grabber_bottom_end = grabber_location + grabber_end_location_from_arm_axis;

    // Location of the claw horizontal axis of rotation relative to the
    // arm axis of rotation with the elevator at 0 and the arm angle of 0
    // The horizontal axis is the up and down motion axis.
    Eigen::Vector2d claw_updown_axis(geometry_.grabber_arm_horz_separation,
                                     -geometry_.grabber_arm_vert_separation);

    // This point is used to make a cross product with the bottom end of the
    // grabber
    // The result of the cross product tells if the parts intersect or not.
    Eigen::Vector2d claw_top_ref_point =
        claw_updown_axis + geometry_.claw_top_thickness * claw_j_unit_direction;

    Eigen::Vector2d claw_top_ref_point_to_grabber_bottom_end =
        grabber_bottom_end - claw_top_ref_point;
    double claw_grabber_check =
        Cross(claw_i_unit_direction, claw_top_ref_point_to_grabber_bottom_end);

    // Now set the safe claw angle.
    if (claw_grabber_check > 0.0) {
      *safe_claw_angle = claw_angle;
      return true;
    } else if (grabber_bottom_end.y() <
               claw_updown_axis.y() +
                   geometry_.claw_top_thickness) {  // grabber is too close
      *safe_claw_angle = -1.0;
      return false;
    } else {
      //  To find the safe angle for the claw, draw a line between the claw
      //  rotation axis and the lower front corner of the grabber.  The angle of
      //  this line is used with the angle between the edge of the claw and the
      //  center line of the claw to determine the angle of the claw.
      Eigen::Vector2d claw_axis_to_grabber_bottom_end =
          grabber_bottom_end - claw_updown_axis;
      double hypot = claw_axis_to_grabber_bottom_end.norm();
      double angleDiff = ::std::asin(geometry_.claw_top_thickness / hypot);
      *safe_claw_angle = ::std::atan2(claw_axis_to_grabber_bottom_end.y(),
                                      claw_axis_to_grabber_bottom_end.x()) -
                         angleDiff;
      return false;
    }
  }

  double get_elevator_min() { return elevator_min_; }

  double get_elevator_max() { return elevator_max_; }

  double get_upper_angle_limit() { return upper_angle_limit_; }

  double get_lower_angle_limit() { return lower_angle_limit_; }

 private:
  // length of the arm
  double length_arm_;
  // max height the elevator can go.
  double elevator_max_;
  // min height the elevator can go.
  double elevator_min_;
  // arm angle upper limit
  double upper_angle_limit_;
  // arm angle lower limit
  double lower_angle_limit_;
  // Geometry of the arm + fridge
  frc971::constants::Values::ClawGeometry geometry_;
};

}  // namespace util
}  // namespace aos

#endif  // Y2015_UTIL_KINEMATICS_H_
