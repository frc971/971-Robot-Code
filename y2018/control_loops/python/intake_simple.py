#!/usr/bin/python3

# This code was used to select the gear ratio for the intake.
# Run it from the command line and it displays the time required
# to rotate the intake 180 degrees.
#
# Michael Schuh
# January 20, 2018

import math
import numpy
import scipy.integrate

pi = math.pi
pi2 = 2.0 * pi
rad_to_deg = 180.0 / pi
inches_to_meters = 0.0254
lbs_to_kg = 1.0 / 2.2
newton_to_lbf = 0.224809
newton_meters_to_ft_lbs = 0.73756
run_count = 0
theta_travel = 0.0

def to_deg(angle):
  return angle * rad_to_deg

def to_rad(angle):
  return angle / rad_to_deg

def to_rotations(angle):
  return angle / pi2

def time_derivative(x, t, voltage, c1, c2, c3):
  global run_count
  theta, omega = x
  dxdt = [omega, -c1 * omega + c3 * math.sin(theta) + c2 * voltage]
  run_count = run_count + 1

  return dxdt

def get_distal_angle(theta_proximal):
  # For the proximal angle = -50 degrees, the distal angle is -180 degrees
  # For the proximal angle =  10 degrees, the distal angle is  -90 degrees
  distal_angle = to_rad(-180.0 - (-50.0 - to_deg(theta_proximal)) * \
      (180.0 - 90.0) / (50.0 + 10.0))
  return distal_angle


def get_180_degree_time(c1, c2, c3, voltage, gear_ratio, motor_free_speed):
  global run_count
  global theta_travel

  if ( True ):
    # Gravity is assisting the motion.
    theta_start = 0.0
    theta_target = pi
  elif ( False ):
    # Gravity is assisting the motion.
    theta_start = 0.0
    theta_target = -pi
  elif ( False ):
    # Gravity is slowing the motion.
    theta_start = pi
    theta_target = 0.0
  elif ( False ):
    # Gravity is slowing the motion.
    theta_start = -pi
    theta_target = 0.0
  elif ( False ):
    # This is for the proximal arm motion.
    theta_start = to_rad(-50.0)
    theta_target = to_rad(10.0)

  theta_half = 0.5 * (theta_start + theta_target)
  if theta_start > theta_target:
    voltage = -voltage
  theta = theta_start
  theta_travel = theta_start - theta_target
  if run_count == 0:
    print("# Theta Start = %.2f radians End = %.2f Theta travel %.2f "
          "Theta half = %.2f Voltage = %.2f" % (
              theta_start, theta_target, theta_travel, theta_half, voltage))
    print("# Theta Start = %.2f degrees End = %.2f Theta travel %.2f "
          "Theta half = %.2f Voltage = %.2f" % (
              to_deg(theta_start), to_deg(theta_target), to_deg(theta_travel),
              to_deg(theta_half), voltage))
  omega = 0.0
  time = 0.0
  delta_time = 0.01 # time step in seconds
  for step in range(1, 5000):
     t = numpy.array([time, time + delta_time])
     time = time + delta_time
     x = [theta, omega]
     angular_acceleration = -c1 * omega + c2 * voltage
     x_n_plus_1 = scipy.integrate.odeint(time_derivative, x, t,
                                         args=(voltage, c1, c2, c3))
     theta, omega = x_n_plus_1[1]

     if ( False ):
       print("%4d  %8.4f %8.2f          %8.4f          %8.4f    %8.3f    "
             "%8.3f     %8.3f      %8.3f" % (
                 step, time, theta, omega, angular_acceleration,
                 to_rotations(theta), to_rotations(omega),
                 omega * gear_ratio * 60.0 / pi2,
                 omega * gear_ratio / motor_free_speed))
     if theta_start < theta_target:
       # Angle is increasing through the motion.
       if theta > theta_half:
         break
     else:
       # Angle is decreasing through the motion.
       if (theta < theta_half):
         break

  return 2.0 * time

def main():
  # m/sec^2 Gravity Constant
  gravity = 9.8
  # m/sec^2 Gravity Constant - Use 0.0 for the intake.  It is horizontal.
  gravity = 0.0
  # Volts
  voltage_nominal = 12

  # Vex 775 Pro motor specs from http://banebots.com/p/M2-RS550-120
  motor_name = "Vex 775 Pro motor specs from http://banebots.com/p/M2-RS550-120"
  current_stall = 134 # amps stall current
  current_no_load = 0.7 # amps no load current
  torque_stall = 710/1000.0 # N-m Stall Torque
  speed_no_load_rpm = 18730 # RPM no load speed

  if ( True ):
    # Bag motor from https://www.vexrobotics.com/217-3351.html
    motor_name = "Bag motor from https://www.vexrobotics.com/217-3351.html"
    current_stall = 53.0 # amps stall current
    current_no_load = 1.8 # amps no load current
    torque_stall = 0.4 # N-m Stall Torque
    speed_no_load_rpm = 13180.0 # RPM no load speed

  if ( False ):
    # Mini CIM motor from https://www.vexrobotics.com/217-3371.html
    motor_name = "Mini CIM motor from https://www.vexrobotics.com/217-3371.html"
    current_stall = 89.0 # amps stall current
    current_no_load = 3.0 # amps no load current
    torque_stall = 1.4 # N-m Stall Torque
    speed_no_load_rpm = 5840.0 # RPM no load speed

  # How many motors are we using?
  num_motors = 1

  # Motor values
  print("# Motor: %s" % (motor_name))
  print("# Number of motors: %d" % (num_motors))
  print("# Stall torque: %.1f n-m" % (torque_stall))
  print("# Stall current: %.1f amps" % (current_stall))
  print("# No load current: %.1f amps" % (current_no_load))
  print("# No load speed: %.0f rpm" % (speed_no_load_rpm))

  # Constants from motor values
  resistance_motor = voltage_nominal / current_stall
  speed_no_load_rps = speed_no_load_rpm / 60.0 # Revolutions per second no load speed
  speed_no_load = speed_no_load_rps * 2.0 * pi
  Kt = num_motors * torque_stall / current_stall # N-m/A torque constant
  Kv_rpm = speed_no_load_rpm / (voltage_nominal -
                                resistance_motor * current_no_load)  # rpm/V
  Kv = Kv_rpm * 2.0 * pi / 60.0  # rpm/V

  # Robot Geometry and physics
  # m Length of arm connected to the robot base
  length_proximal_arm = inches_to_meters * 47.34
  # m Length of arm that holds the cube
  length_distal_arm = inches_to_meters * 44.0
  # m Length of intake arm from the pivot point to where the big roller contacts a cube.
  length_intake_arm =  inches_to_meters * 9.0
  mass_cube = 6.0 * lbs_to_kg  # Weight of the cube in Kgrams
  mass_proximal_arm = 5.5 * lbs_to_kg # Weight of proximal arm
  mass_distal_arm = 3.5 * lbs_to_kg # Weight of distal arm
  mass_distal = mass_cube + mass_distal_arm
  mass_proximal = mass_proximal_arm + mass_distal
  # m Length from arm pivot point to arm CG
  radius_to_proximal_arm_cg = 22.0 * inches_to_meters
  # m Length from arm pivot point to arm CG
  radius_to_distal_arm_cg = 10.0 * inches_to_meters

  radius_to_distal_cg = (length_distal_arm * mass_cube +
                         radius_to_distal_arm_cg * mass_distal_arm) / \
                             mass_distal
  radius_to_proximal_cg = (length_proximal_arm * mass_distal +
                           radius_to_proximal_arm_cg * mass_proximal_arm) / \
                               mass_proximal
  J_cube = length_distal_arm * length_distal_arm*mass_cube
  # Kg m^2 Moment of inertia of the proximal arm
  J_proximal_arm = radius_to_proximal_arm_cg * radius_to_proximal_arm_cg * \
      mass_distal_arm
  # Kg m^2 Moment of inertia distal arm and cube at end of proximal arm.
  J_distal_arm_and_cube_at_end_of_proximal_arm = length_proximal_arm * \
      length_proximal_arm * mass_distal
  # Kg m^2 Moment of inertia of the distal arm
  J_distal_arm = radius_to_distal_arm_cg * radius_to_distal_arm_cg * mass_distal_arm
  # Moment of inertia of the arm with the cube on the end
  J = J_distal_arm_and_cube_at_end_of_proximal_arm + J_proximal_arm
  # Intake claw
  J_intake = 0.295  # Kg m^2 Moment of inertia of intake
  J = J_intake

  gear_ratio = 140.0  # Guess at the gear ratio
  gear_ratio = 100.0  # Guess at the gear ratio
  gear_ratio = 90.0  # Guess at the gear ratio

  error_margine = 1.0
  voltage = 10.0  # voltage for the motor.  Assuming a loaded robot so not using 12 V.
  # It might make sense to use a lower motor frees peed when the voltage is not a full 12 Volts.
  # motor_free_speed = Kv * voltage
  motor_free_speed = speed_no_load

  print("# Kt = %f N-m/A\n# Kv_rpm = %f rpm/V\n# Kv = %f radians/V" % (Kt, Kv_rpm, Kv))
  print("# %.2f Ohms Resistance of the motor " % (resistance_motor))
  print("# %.2f kg Cube weight" % (mass_cube))
  print("# %.2f kg Proximal Arm mass" % (mass_proximal_arm))
  print("# %.2f kg Distal Arm mass" % (mass_distal_arm))
  print("# %.2f kg Distal Arm and Cube weight" % (mass_distal))
  print("# %.2f m Length from distal arm pivot point to arm CG" % (
      radius_to_distal_arm_cg))
  print("# %.2f m Length from distal arm pivot point to arm and cube cg" % (
      radius_to_distal_cg))
  print("# %.2f kg-m^2 Moment of inertia of the cube about the arm pivot point" % (J_cube))
  print("# %.2f m Length from proximal arm pivot point to arm CG" % (radius_to_proximal_arm_cg))
  print("# %.2f m Length from proximal arm pivot point to arm and cube cg" % (
      radius_to_proximal_cg))
  print("# %.2f m  Proximal arm length" % (length_proximal_arm))
  print("# %.2f m  Distal arm length" % (length_distal_arm))

  print("# %.2f kg-m^2 Moment of inertia of the intake about the intake pivot point" % (
      J_intake))
  print("# %.2f kg-m^2 Moment of inertia of the distal arm about the arm pivot point" % (
      J_distal_arm))
  print("# %.2f kg-m^2 Moment of inertia of the proximal arm about the arm pivot point" % (
      J_proximal_arm))
  print("# %.2f kg-m^2 Moment of inertia of the distal arm and cube mass about "
        "the proximal arm pivot point" % (
            J_distal_arm_and_cube_at_end_of_proximal_arm))
  print("# %.2f kg-m^2 Moment of inertia of the intake the intake pivot point "
        "(J value used in simulation)" % (J))
  print("# %d Number of motors" % (num_motors))

  print("# %.2f V Motor voltage" % (voltage))
  for gear_ratio in range(60, 241, 10):
    c1 = Kt * gear_ratio * gear_ratio / (Kv * resistance_motor * J)
    c2 = gear_ratio * Kt / (J * resistance_motor)
    c3 = radius_to_proximal_cg * mass_proximal * gravity / J

    if ( False ):
      print("# %.8f 1/sec C1 constant" % (c1))
      print("# %.2f 1/sec C2 constant" % (c2))
      print("# %.2f 1/(V sec^2) C3 constant" % (c3))
      print("# %.2f RPM Free speed at motor voltage" % (voltage * Kv_rpm))

    torque_90_degrees = radius_to_distal_cg * mass_distal * gravity
    voltage_90_degrees = resistance_motor * torque_90_degrees / (gear_ratio * Kt)
    torque_peak = gear_ratio * num_motors * torque_stall
    torque_peak_ft_lbs = torque_peak * newton_meters_to_ft_lbs
    normal_force = torque_peak / length_intake_arm
    normal_force_lbf = newton_to_lbf * normal_force
    time_required = get_180_degree_time(c1, c2, c3, voltage,
                                        gear_ratio, motor_free_speed)
    print("Time for %.1f degrees for gear ratio %3.0f is %.2f seconds.  "
          "Peak (stall) torque %3.0f nm %3.0f ft-lb Normal force at intake "
          "end %3.0f N %2.0f lbf" % \
      (to_deg(theta_travel), gear_ratio, time_required,
       torque_peak, torque_peak_ft_lbs, normal_force, normal_force_lbf))

if __name__ == '__main__':
  main()
