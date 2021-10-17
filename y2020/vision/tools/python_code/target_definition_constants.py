# Using coordinate system as defined in sift.fbs:
# field origin (0, 0, 0) at floor height at center of field
# Robot orientation with x-axis pointing towards RED ALLIANCE player station
# y-axis to left and z-axis up (right-hand coordinate system)
# Thus, the red power port target location will be at (-15.98/2,1.67,0),
# with orientation (facing out of the field) of M_PI

# Field constants for target definitions
# We have split these out into a separate file for now because they are not being
# used in target_definition, but they may be useful in the future.
field_length = 15.983
power_port_total_height = 3.10
power_port_edge_y = 1.089
power_port_width = 1.225
power_port_bottom_wing_height = 1.828
power_port_wing_width = 1.83
loading_bay_edge_y = 0.784
loading_bay_width = 1.524
loading_bay_height = 0.933
# Wing angle on actual field target
# wing_angle = 20. * math.pi / 180.
### NOTE: Setting wing angle to zero to match current FRC971 target
wing_angle = 0

# Pick the target center location at halfway between top and bottom of the top panel
power_port_target_height = (power_port_total_height +
                            power_port_bottom_wing_height) / 2.

### Taped up FRC target definition
inch_to_meter = 0.0254
c_power_port_total_height = (79.5 + 39.5) * inch_to_meter
c_power_port_edge_y = 1.089
c_power_port_width = 4.0 * 12 * inch_to_meter
c_power_port_bottom_wing_height = 79.5 * inch_to_meter
c_power_port_wing_width = 47.5 * inch_to_meter
c_power_port_white_marker_z = (79.5 - 19.5) * inch_to_meter

# Pick the target center location at halfway between top and bottom of the top panel
c_power_port_target_height = (power_port_total_height +
                              power_port_bottom_wing_height) / 2.
