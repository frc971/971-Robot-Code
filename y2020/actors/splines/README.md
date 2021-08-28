# Spline Descriptions
This folder contains reference material for what each spline does

<br>

# Target Aligned - Infinite Recharge
## After shooting balls initially, collects power cells and returns to shooting position

### [Target Aligned 1](target_aligned_1.json)
After shooting the pre-loaded balls, this spline directs the robot to the power cells, allowing the robot to collect all three balls available

### [Target Aligned 2](target_aligned_2.json)
After collecting the 3 balls using the target_aligned_1 spline, this spline directs the robot back towards the shooter tower, setting it up to shoot the balls it collected

<br>

# Target Offset - Infinite Recharge
## After starting with 3 balls, collects power cells and heads to primary shooting position

### [Target Offset 1](target_offset_1.json)
When we start offset from the target with the 3 pre-loaded balls in the robot, this spline directs the robot to the additional 2 power cells, allowing the robot to collect both balls available

### [Target Offset 2](target_offset_2.json)
After collecting the 2 balls using the target_offset_1 spline, this spline directs the robot towards the shooter tower, setting it up to shoot the 5 balls.