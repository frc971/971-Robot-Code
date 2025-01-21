Spline UI
================================================================================

The Spline UI is a locally hosted web app with an AngularJS frontend and Flask backend. It's used to plan custom swerve paths and actions for autonomous. Paths are modeled as one or more connected splines.

## Running the app

```console
bazel run //frc971/control_loops/swerve/spline_ui:server
```

Then, navigate to <http://localhost:1180> to look at the web page.

## Description

### Parts of a path

* Continuous splines: continuous motion without stopping
* Stop point splines: similar to continuous splines but the robot stops and can change direction at the start point
* Rotation breakpoints: set the robot's orientation at specific points along the path
* Constraints: apply restrictions to specific portions of the path
* Events/actions: trigger predefined robot actions (e.g. "Intake" from `y2024_bot3/autonomous/autonomous_main.cc`)

### Features

* Drag to adjust splines, rotation breakpoints, actions, and constraints
* Modify the metadata of selected constraints and actions in the right sidebar
* **Save paths as JSON files and load them later**
* Solve paths using the trajectory solver and view the data in the graph
* View the robot's trajectory over time and play it at adjustable speeds after solving the trajectory using the timeline

### Workflow: From Path to Robot Directions

1. Draw a path on the Spline UI, and save it as a JSON file using "save path" button
2. Copy the JSON file to `y2024_bot3/autonomous/paths`
3. Edit the `genrule_trajectory` genrule in `y2024_bot3/autonomous/BUILD` to take in the JSON filepath in `srcs`
    * this genrule runs `//frc971/control_loops/swerve/spline_ui:generate_solved_trajectory` which runs `//frc971/control_loops/swerve/spline_ui:trajectory_solver`, turning our spline JSON into trajectory data (with position, velocity, and acceleration, over time, in `trajectory.json`)
4. Build and deploy code onto the robot
5. When the robot is enabled, `//y2024_bot3/autonomous:autonomous_main` will run (this is configured in `y2024_bot3_roborio.json`)
    * `autonomous_main` creates a `frc971::control_loops::swerve::AutonomousController`
    * when the event loop goes through `AutonomousController::Iterate`, it generates velocity goals for the swerve drivetrain based on the data in `trajectory.json`, in real time
