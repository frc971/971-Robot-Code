export type StopPoint = { next_spline_index: number, delay: number | null }

export type GlobalConstraint = {
  max_velocity: number | null
  max_acceleration: number | null
  max_current: number | null
  max_voltage: number | null
}

export type Constraint = {
  selection: [number, number] // constraint starts at selection[0] and ends at selection[1]
} & GlobalConstraint

export type ActionInfo = {
  location: number,
  name: string
}

export type Rotation = {
  location: number,
  angle: number // radians
}

// This type represents the exact result of the trajectory solver.
// See SwerveSolution in swerve_trajectory_optimizer.py for explanation of these properties.
export type OptiData = {
  driving_currents: [number, number, number, number][],
  voltages: [number, number, number, number][],
  lat_forces: [number, number, number, number][],
  positions: [number, number, number][],
  velocities: [number, number, number][],
  accelerations: [number, number, number][],
  module_forces: [[number, number], [number, number], [number, number], [number, number]][]
  ang_vels: [number, number, number, number][]
  times: number[],
  mod_vels: [[number, number], [number, number], [number, number], [number, number]][]
}

type SplineLinkType = "G1" | "G0"

export type Point = [number, number]

export type SplineControl = [Point, Point, Point, Point, Point, Point]

export type Spline = {
  control_points: SplineControl,
  curve_points: Point[],
  next_spline?: Spline,
  next_spline_link?: SplineLinkType,
  prev_spline?: Spline,
  prev_spline_link?: SplineLinkType,
}
