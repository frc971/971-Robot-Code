import casadi

from frc971.control_loops.python.swerve_trajectory_optimizer import SwervePath, SwerveSolution


def solve(paths, global_constraints) -> list[dict]:
    """
    Solves for the optimal swerve trajectory

    Parameters:
        paths: Specifies the path the robot travels. A dict in this format:
            {
                splines: [number, number][][]
                rot_breakpoints: [number, number][]
                constraints: {
                    selection: [number, number],
                    max_voltage: number | null
                    max_current: number | null
                    max_acceleration: number | null
                    max_velocity: number | null
                }[]
            }[]
        global_constraints: Specifies the constraints on how the robot can move.
            Gets overridden by the constraints in paths. A dict of format:
            {
                max_voltage: number | null
                max_current: number | null
                max_acceleration: number | null
                max_velocity: number | null
            }
    """

    print("Requested paths: ", paths)
    solved_trajectories = []
    spline_index = 0
    for path in paths:
        splines = path["splines"]
        spline_paths = [
            SwervePath.spline_path(spline, 0, 0) for spline in splines
        ]
        combined_path: SwervePath = SwervePath.combine_paths(*spline_paths)
        rot_breakpoints = path["rot_breakpoints"]
        theta = casadi.MX.sym('theta')  # Position on the path

        eq = 0  # Rotation of the path as a function of the current position theta
        for i in range(len(rot_breakpoints) - 1):
            # Only add this interval (with sel = 1) if theta is between the breakpoints
            rbp1 = rot_breakpoints[i]
            rbp2 = rot_breakpoints[i + 1]
            sel = casadi.if_else(theta <= rbp2[0], 1, 0)
            if i != 0:
                sel = sel * casadi.if_else(theta > rbp1[0], 1, 0)

            # Normalize theta to be a fraction of the position interval
            t = (theta - rbp1[0]) / (rbp2[0] - rbp1[0])
            # Scaling because the integral of the cubic profile function over [0, 1] is 1/2240
            A = 1120 * (rbp2[1] - rbp1[1])
            # Calculate the cubic angular acceleration profile for the interval
            # Ensure we start and end at 0 angular acceleration and velocity
            eq += sel * (A * (t**7 / 42 - t**6 / 12 + 9 * t**5 / 80 -
                              7 * t**4 / 96 + t**3 / 48) + rbp1[1])

        combined_path.control_rot(eq, theta)

        max_acceleration_sections = []
        max_velocity_sections = []
        max_voltage_sections = []
        max_current_sections = []
        for constraint in path["constraints"]:
            max_acceleration_sections.append(
                (*constraint["selection"], constraint["max_acceleration"]))
            max_velocity_sections.append(
                (*constraint["selection"], constraint["max_velocity"]))
            max_voltage_sections.append(
                (*constraint["selection"], constraint["max_voltage"]))
            max_current_sections.append(
                (*constraint["selection"], constraint["max_current"]))

        solved_trajectory = SwerveSolution(
            200 * len(spline_paths),
            combined_path,
            max_acceleration=global_constraints["max_acceleration"],
            max_velocity=global_constraints["max_velocity"],
            max_voltage=global_constraints["max_voltage"],
            max_current=global_constraints["max_current"],
            max_acceleration_sections=max_acceleration_sections,
            max_current_sections=max_current_sections,
            max_velocity_sections=max_velocity_sections,
            max_voltage_sections=max_voltage_sections,
            solver_print=True)
        print("Solved path:", path)
        print("Solved trajectory:", solved_trajectory)
        # All the list()s are to get rid of np.ndarrays so it can jsonify nicely
        sol_dict = {
            "driving_currents":
            list(list(i) for i in solved_trajectory.driving_currents),
            "voltages":
            list(list(i) for i in solved_trajectory.voltages),
            "velocities":
            list(list(i) for i in solved_trajectory.velocities),
            "accelerations":
            list(list(i) for i in solved_trajectory.accelerations),
            "positions":
            list(list(i) for i in solved_trajectory.positions),
            "times":
            list(solved_trajectory.times),
            "module_forces":
            list(list(i) for i in solved_trajectory.module_forces),
            "lat_forces":
            list(list(i) for i in solved_trajectory.lat_forces),
            "ang_vels":
            list(list(i) for i in solved_trajectory.ang_vels),
            "mod_vels": [
                list(
                    list(list(j) for j in i)
                    for i in solved_trajectory.mod_vels)
            ][0],
            "spline_range": [spline_index, spline_index + len(splines)],
        }
        solved_trajectories.append(sol_dict)
        spline_index += len(splines)
    return solved_trajectories


def solve_and_discretize(saved_path) -> tuple[list[dict], list[dict]]:
    """
    Uses linear interpolation to get the position, velocity, and acceleration of the robot every 5 milliseconds.
    Accounts for startDelays and calculates actions' timestamps.
    Note that saved_path NOT what solve() takes, but the direct saved json from SplineUi
    Returns a tuple of the discretized trajectory and actions in the format:
    <pre>(
        {
            time: number (seconds),
            position: [x, y, theta],
            velocity: [x, y, theta],
            acceleration: [x, y, theta]
        }[],
        {
            time: number (seconds),
            name: string
        }[]
    )</pre>
    """

    paths = saved_path["paths"]
    global_constraints = saved_path["global_constraints"]
    actions = saved_path["actions"]
    solved_trajectories = solve(paths, global_constraints)

    discretized_trajectory = []

    path_index = 0
    last_trajectory_time = 0
    last_bounding_point_index = 0

    trajectory = solved_trajectories[path_index]
    path = paths[path_index]
    # Convert t to seconds and make it relative to the start of the current trajectory
    # Run the loop for 15000 ms (15 s)
    for t in range(0, 15000 + 5, 5):
        time_in_trajectory = t / 1000 - last_trajectory_time - path[
            "startDelay"]

        # This is a while and not an if just in case we have <5ms trajectories (you never know...)
        while time_in_trajectory > trajectory["times"][-1]:
            # We've gone past the end of the trajectory - switch to the next one
            path_index += 1

            # Check if we're done with all the trajectories
            if path_index >= len(solved_trajectories):
                break

            # Add to the total time taken by up to the last trajectory
            last_trajectory_time = last_trajectory_time + trajectory["times"][
                -1] + path["startDelay"]
            last_bounding_point_index = 0

            trajectory = solved_trajectories[path_index]
            path = paths[path_index]
            time_in_trajectory = t / 1000 - last_trajectory_time - path[
                "startDelay"]

        if path_index >= len(solved_trajectories):
            # Don't move if done with trajectories
            discretized_trajectory.append({
                "time":
                t / 1000,
                "position":
                [*path["splines"][-1][-1], path["rot_breakpoints"][-1][1]
                 ],  # Last control point is where it waits
                "velocity": [0, 0, 0],
                "acceleration": [0, 0, 0]
            })

        elif time_in_trajectory < 0:
            # It doesn't move if its in a delay
            discretized_trajectory.append({
                "time":
                t / 1000,
                "position":
                [*path["splines"][0][0], path["rot_breakpoints"][0][1]
                 ],  #first control point is where it waits
                "velocity": [0, 0, 0],
                "acceleration": [0, 0, 0]
            })

        else:
            # Get the two bounding points of the trajectory
            while trajectory["times"][
                    last_bounding_point_index] <= time_in_trajectory:
                last_bounding_point_index += 1

            end_time = trajectory["times"][last_bounding_point_index]
            end_position = trajectory["positions"][last_bounding_point_index]
            end_velocity = trajectory["velocities"][last_bounding_point_index]

            if last_bounding_point_index != 0:
                start_time = trajectory["times"][last_bounding_point_index - 1]
                start_position = trajectory["positions"][
                    last_bounding_point_index - 1]
                start_velocity = trajectory["velocities"][
                    last_bounding_point_index - 1]
                average_acceleration = trajectory["accelerations"][
                    last_bounding_point_index - 1]
            else:
                discretized_trajectory.append({
                    "time":
                    t / 1000,
                    "position":
                    end_position,
                    "velocity":
                    end_velocity,
                    "acceleration":
                    trajectory["accelerations"][0]
                })
                continue

            # To clarify how these are interpolated:
            # Acceleration is (mostly) held constant throughout an interval by the solver (barring extreme curvatures to the path)
            # So it just takes what the solver gives it
            # Velocity is (mostly) linear due to the acceleration being (mostly) constant, so it is just lerped
            # Position is integrated up from the velocity function, but to prevent the ends not matching up
            # (because the acceleration does slightly change over time)
            # a weighted average is taken which values the shorter integrals

            # Lerp between the trajectories
            start_factor = (end_time - time_in_trajectory) / (end_time -
                                                              start_time)
            end_factor = (time_in_trajectory - start_time) / (end_time -
                                                              start_time)

            lerped_velocity = [
                start_velocity_component * start_factor +
                end_velocity_component * end_factor
                for start_velocity_component, end_velocity_component in zip(
                    start_velocity, end_velocity)
            ]

            # Weighted average of the integral of the velocity from both sides
            t0 = start_time
            t1 = end_time
            start_pos_integral = [
                p0 + (v0 * t1 - v1 * t0) / (t1 - t0) *
                (time_in_trajectory - t0) + (v1 - v0) / (t1 - t0) *
                (time_in_trajectory**2 - t0**2) / 2 for p0, v0, v1 in zip(
                    start_position, start_velocity, end_velocity)
            ]
            end_pos_integral = [
                p1 + (v0 * t1 - v1 * t0) / (t1 - t0) *
                (time_in_trajectory - t1) + (v1 - v0) / (t1 - t0) *
                (time_in_trajectory**2 - t1**2) / 2 for p1, v0, v1 in zip(
                    end_position, start_velocity, end_velocity)
            ]

            estimated_position = [
                start_pos_integral_component * start_factor +
                end_pos_integral_component * end_factor
                for start_pos_integral_component, end_pos_integral_component in
                zip(start_pos_integral, end_pos_integral)
            ]

            discretized_trajectory.append({
                "time": t / 1000,
                "position": estimated_position,
                "velocity": lerped_velocity,
                "acceleration": average_acceleration
            })

    timestamped_actions = []
    total_trajectory_indicies = sum(
        [len(trajectory["times"]) for trajectory in solved_trajectories])

    for action in actions:
        # The 'location' of an action refers to its position along the splines normalized from 0 to 1
        # This means that the index of it can be easily determined because the splines are discretized the same way
        # (although the index may be fractional, requiring more lerping)
        index = action["location"] * total_trajectory_indicies

        path_index = 0
        last_trajectory_time = 0

        while index > len(solved_trajectories[path_index]["times"]):
            index -= len(solved_trajectories[path_index]["times"])
            last_trajectory_time += solved_trajectories[path_index]["times"][
                -1] + paths[path_index]["startDelay"]
            path_index += 1

        start_index = int(index)
        end_index = start_index + 1

        if end_index == len(solved_trajectories[path_index]["times"]):
            timestamped_actions.append({
                "time":
                solved_trajectories[path_index]["times"][-1],
                "name":
                action["name"]
            })
        else:
            timestamped_actions.append({
                "time":
                paths[path_index]["startDelay"] + last_trajectory_time +
                solved_trajectories[path_index]["times"][start_index] *
                (end_index - index) +
                solved_trajectories[path_index]["times"][end_index] *
                (index - start_index),
                "name":
                action["name"]
            })

    return (discretized_trajectory, timestamped_actions)
