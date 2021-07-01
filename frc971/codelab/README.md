# FRC971 "Codelab"

Welcome! This folder contains a "codelab" where you can go through the process
of fleshing out a basic control-loop using the same infrastructure as we do for
the control loops that normally run on our robots. Currently, this just consists
of a single codelab; the instructions can be found below.

## Setup

Before starting this codelab, you should already have set up your computer with all of the programs needed to build and run the robot code, or have an account on the build server. If you have not done this, follow the instructions in the [introduction](https://software.frc971.org/gerrit/plugins/gitiles/971-Robot-Code/+/refs/heads/master/README.md).

## Flatbuffers tutorial

Our code uses flatbuffers extensively. If you're unfamiliar with them, you can take a look at these [tutorials](https://google.github.io/flatbuffers/flatbuffers_guide_tutorial.html) for how to use them.  This is optional but reommended if you are looking for more background on flatbuffers, and can be done before or after the codelab.

## Instructions

This codelab helps build basic knowledge of how to use 971 control loop
primatives.

When this codelab is run, it performs a series of tests to check whether the code is working properly. Your job is to add or make changes to the code to get the tests to pass.

### Running the tests

In order to run the tests, execute the following command in the terminal in the 971-Robot-Code folder:
```
bazel run frc971/codelab:basic_test -- --gtest_color=yes
```

In total, there are 7 tests, 3 of which will fail when you first run them. As each tests fails, it will print out the details of how each test result differed from the expected value. At the bottom, it will summarize the number of passed and failed tests.

As you work through the codelab, you can run the tests to check your progress. Once they all pass, you are finished and can move on to submitting a code review.

### Control loops

A control loop is a piece of code that is repeatedly executed while the robot is running, recieiving input from the robot controllers and sensors and sending intructions to the motors that control the robot.

Control loops all follow the same structure:
There are 4 channels that send and recieve instructions. These channels are goal, position, output, and status. Goal and position are input channels, which recieve messages from the robot's sensors and input from the controller. Output and status are output channels, which send messages to the motors.

::frc971::controls::ControlLoop is a helper class that takes
all the channel types as template parameters and then calls
RunIteration() whenever a Position message is received.
It will pass in the Position message and most recent Goal message
and provide Builders that the RunIteration method should use to
construct and send output/status messages.

The various basic_*.fbs files define the Goal, Position, Status, and Output
messages. The code for the tests is in the basic_test.cc file, and the code you will edit is in the basic.cc file.

In order to get the tests to pass, you'll need to fill out the RunIteration()
implementation in basic.cc so that it uses the input goal/position to
meaningfully populate the output/status messages. You can find descriptions
of exactly what the fields of the messages mean by reading all the *.fbs
files, and the tests below can be reviewed to help understand exactly what
behavior is expected.

### Submitting a code review

Once you can get the tests to pass, follow the directions in [this file](https://software.frc971.org/gerrit/plugins/gitiles/971-Robot-Code/+/refs/heads/master/documentation/tutorials/submitting-code-for-a-review.md) for creating a
code review of the change. We will not actually *submit* the change (since
that  would remove the challenge for future students), but we will go through
the code review process.