# 3128-common

Code that Team 3128, the Aluminum Narwhals, shares between robots: mechanisms, state machines,
motor controllers, vision, logging and dashboards. Each season's robot project builds on it instead
of starting from scratch.

## What's in it

| Package | What it's for |
|---|---|
| [`common.core.subsystems`](src/main/java/common/core/subsystems/README.md) | **Mechanisms.** `LinearMechanism` (elevators, sliders), `PivotMechanism` (arms, wrists, hoods, turrets) and `RollerMechanism` (intakes, flywheels). Start here. |
| [`common.core.fsm`](src/main/java/common/core/fsm/README.md) | **State machines.** Subsystems with named states like NEUTRAL and INTAKE, and rules for moving between them. |
| [`common.core.controllers`](src/main/java/common/core/controllers/README.md) | `PIDFFConfig`, which holds a mechanism's gains. Its guide explains what each gain does. |
| [`common.hardware.motorcontroller`](src/main/java/common/hardware/motorcontroller/README.md) | TalonFX and Spark motor controllers behind one interface, plus physics models for simulation. |
| [`common.hardware.camera`](src/main/java/common/hardware/camera/README.md) | PhotonVision cameras that find AprilTags and correct the robot's position. |
| [`common.hardware.input`](src/main/java/common/hardware/input/README.md) | Xbox controllers, joysticks and the team's button board. |
| `common.core.misc` | `NAR_Robot`, the base for the robot's `Robot` class. |
| [`common.utility`](src/main/java/common/utility/README.md) | Console logging, the dashboard (Elastic), Shuffleboard, pit system checks and SysId. NarwhalDashboard is still here while the team switches to Elastic. |

## Documentation

Each package folder has a `README.md` that GitHub shows below the file list. Those guides explain
concepts and how the classes fit together. Javadoc on classes and methods describes what each one
does, and appears when you hover over a name in your editor.

When you change code, keep the documentation with it:

- **Concepts and walkthroughs** go in the package's `README.md`.
- **What a class, method or field does** goes in a `/** Javadoc */` comment above it, including
  units and side effects.
- **Why a line of code is the way it is** goes in a `//` comment on the line above, only when the
  code cannot show it, such as a hardware quirk or a physics derivation. Do not restate what the
  code does.
- Do not leave commented-out code. Git keeps the history.
- Write comments formally: complete third-person sentences such as "Returns the current position",
  with no contractions and no "you". Name the subject rather than writing "it" where the subject
  is unclear.

## A mechanism in a few lines

```java
// An arm in degrees: starts hanging straight down, swings from -90 to 90, weighs 4 kg
arm = new PivotMechanism("Arm", Orientation.FORWARD, ARM_CONFIG, ARM_GAINS, 1,
    -90, 90, -90, Math.PI / 180, 0.6, 4, DCMotor.getKrakenX60(1), new NAR_TalonFX(ARM_ID));

arm.setSetpoint(45);   // go to 45 degrees and hold there
```

From those few lines the arm gets:
- a control loop running on the motor controller 1000 times a second, with gravity feedforward
- a Shuffleboard tab for tuning
- logging of everything for AdvantageScope
- a physics model, so it moves in simulation

## How robot projects use it

The robot project includes this repository as a git submodule in `libs/3128-common`, and compiles
its source together with the robot's own code. To get it after cloning a robot project:

```
git submodule update --init
```

The robot project pins the exact commit of 3128-common it was built with. After changing
3128-common, commit there first, then commit the new submodule commit in the robot project.

Older robot projects got it from JitPack instead, through the `3128-common.json` vendordep in this
repository.

## Building and testing

You need a JDK 17. The WPILib installer includes one; any other JDK 17 works too. Gradle downloads
everything else on the first build.

```
./gradlew build      # compile and run the tests
./gradlew test       # just the tests
./gradlew test jacocoTestReport   # the tests, plus a coverage report
```

The coverage report, at `build/reports/jacoco/test/html/index.html`, shows which lines the tests
run, so you can see what a change still needs tests for. Keep it above 90%.

The tests in `src/test` cover the mechanisms, state machines, motor unit conversions,
simulation physics, vision and dashboards, without needing a robot. Run them before committing
changes. If you change how something behaves, add or update a test that shows it.

## Simulation

Mechanisms and cameras simulate themselves; the robot code doesn't need any `if (isSimulation())`.
In the robot project, `Robot.simulationPeriodic()` steps the physics each loop:

```java
NAR_Motor.updateSimulation(0.02);
DynamicCamera.updateSimulation(drivetrain.getState().Pose);
```

Run the robot in simulation with `./gradlew simulateJava` from the robot project.

## Thanks to team 4450
This common repo is based on robotLib made by team 4450. Parts of the state machine code are based
on ShamLib by team 5907.
