# Generated queue code

This tutorial covers what the code that is generated from a .q file looks like.

### The basics
Each `.q` file generates a `.cc` and `.h` pair that can be #include'd in your c++
code.

We're going to use `//frc971/queues:gyro.q` as our example.

## Package declaration
`.q` files usually start with a `package` declaration. The `package` declaration
directly converts to a set of namespace declarations. So

```cpp
package frc971.sensors;
```

generates the namespace definitions

```cpp
namespace frc971 {
namespace sensors {

// All of the code for the queue

}  // namespace sensors
}  // namespace frc971
```

## Message declarations
Each message declared in the queue generates a class that can be instantiated in
your code easily.

For a simple example, lets use the `Uid` message in `//frc971/queues:gyro.q`:

```cpp
message Uid {
  uint32_t uid;
};
```

Let's take a look at the actual class definition that is created. If you ever
want to inspect the code that is generated from a .q file, you can take a look
in `bazel-genfiles` after you build your code. So after running `bazel build
//frc971/queues:gyro` there will be a `bazel-genfiles/frc971/queues/gyro.q.h`
and `bazel-genfiles/frc971/queues/gyro.q.cc`.

Here's the definition of the `Uid` class from
`bazel-genfiles/frc971/queues/gyro.q.h` (comments mine):

```cpp
struct Uid : public ::aos::Message {
  // Used to identify queues uniquely even if they have the same name.
  enum { kQueueLength = 100, kHash = 0x0837c541 };

  // The actual data that we requested be part of the message.
  uint32_t uid;

  // Writes the message to a byte buffer.
  size_t Serialize(char *buffer) const;
  // Reads the message from a byte buffer.
  size_t Deserialize(const char *buffer);
  // Zeroes all of the fields in the message.
  void Zero();
  // Returns the size of message. Not used
  static size_t Size() { return 4 + ::aos::Message::Size(); }
  // Prints the contents of the message to a string that is human readable.
  size_t Print(char *buffer, size_t length) const;

  // Gets information about this message's type.
  static const ::aos::MessageType *GetType();
  static const ::aos::MessageType *DoGetType();

  // Default constructor, zeroes the struct.
  Uid();
  // Value constructor, sets the data at construction.
  Uid(uint32_t uid_in);
  bool EqualsNoTime(const Uid &other) const;
};
```

## Queue declarations
Every `queue` declaration in the `.q` file creates an `aos::Queue` variable that
uses the message type given. So the declaration

```cpp
queue Uid gyro_part_id;
```

will generate a variable in `gyro.q.h`

```cpp
static UNUSED_VARIABLE::aos::Queue<Uid> &gyro_part_id;
```

which can then be used in any file that includes the header file.


## QueueGroup declarations

`queue_group` declares a set of queues and some associated helpers for them.
There are two kinds of `queue_groups`, declarations and aliases. Here's an
example of a `queue_group` definition:

```cpp
queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    IntakeGoal intake;

    // Used to identify a position in the planned set of positions on the arm.
    uint32_t arm_goal_position;
    // If true, start the grab box sequence.
    bool grab_box;

    bool open_claw;
    bool close_claw;

    bool deploy_fork;

    bool hook_release;

    double voltage_winch;

    double open_threshold;

    bool disable_box_correct;

    bool trajectory_override;
  };

  message Status {
    // Are all the subsystems zeroed?
    bool zeroed;

    // If true, any of the subsystems have aborted.
    bool estopped;

    // Status of both intake sides.
    IntakeSideStatus left_intake;
    IntakeSideStatus right_intake;

    ArmStatus arm;

    double filtered_box_velocity;
    uint32_t rotation_state;
  };

  message Position {
    // Values of the series elastic encoders on the left side of the robot from
    // the rear perspective in radians.
    IntakeElasticSensors left_intake;

    // Values of the series elastic encoders on the right side of the robot from
    // the rear perspective in radians.
    IntakeElasticSensors right_intake;

    ArmPosition arm;

    // Value of the beam breaker sensor. This value is true if the beam is
    // broken, false if the beam isn't broken.
    bool claw_beambreak_triggered;
    // Value of the beambreak sensor detecting when the box has hit the frame
    // cutout.
    bool box_back_beambreak_triggered;

    // Distance to the box in meters.
    double box_distance;
  };

  message Output {
    // Voltage sent to the parts on the left side of the intake.
    IntakeVoltage left_intake;

    // Voltage sent to the parts on the right side of the intake.
    IntakeVoltage right_intake;

    // Voltage sent to the motors on the proximal joint of the arm.
    double voltage_proximal;

    // Voltage sent to the motors on the distal joint of the arm.
    double voltage_distal;

    // Voltage sent to the hanger.  Positive pulls the robot up.
    double voltage_winch;

    // Clamped (when true) or unclamped (when false) status sent to the
    // pneumatic claw on the arm.
    bool claw_grabbed;

    // If true, release the arm brakes.
    bool release_arm_brake;
    // If true, release the hook
    bool hook_release;
    // If true, release the forks
    bool forks_release;
  };

  queue Goal goal;
  queue Output output;
  queue Status status;
  queue Position position;
};
```

and aliases look like:

```cpp
queue_group SuperstructureQueue superstructure_queue;
```

The declaration type creates the definition of the queue group and what messages
and queues it contains. The alias type creates a variable with the given name.

This queue group results in the following code in
`bazel-genfiles/y2018/control_loops/superstructure/superstructure.q.h`

```cpp

struct SuperstructureQueue_Goal : public ::aos::Message { /* ... */ }
struct SuperstructureQueue_Output : public ::aos::Message { /* ... */ }
struct SuperstructureQueue_Status : public ::aos::Message { /* ... */ }
struct SuperstructureQueue_Position : public ::aos::Message { /* ... */ }
class SuperstructureQueue : public ::aos::QueueGroup {
 public:
  typedef SuperstructureQueue_Goal Goal;
  ::aos::Queue<SuperstructureQueue_Goal> goal;
  typedef SuperstructureQueue_Output Output;
  ::aos::Queue<SuperstructureQueue_Output> output;
  typedef SuperstructureQueue_Status Status;
  ::aos::Queue<SuperstructureQueue_Status> status;
  typedef SuperstructureQueue_Position Position;
  ::aos::Queue<SuperstructureQueue_Position> position;
  SuperstructureQueue(const char *name, uint32_t hash, const char *goal_name,
                      const char *output_name, const char *status_name,
                      const char *position_name);
};
```

and a definition of a variable:

```cpp
static UNUSED_VARIABLE SuperstructureQueue &superstructure_queue;
```
