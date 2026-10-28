# 3128-common

Code that Team 3128, the Aluminum Narwhals, shares between robots: mechanisms, state machines,
motor controllers, vision, logging and dashboards. Each season's robot project builds on it instead
of starting from scratch.

## What's in it

| Package | What it's for |
|---|---|
| `common.core.subsystems` | **Mechanisms.** `LinearMechanism` (elevators, sliders), `PivotMechanism` (arms, wrists, hoods, turrets) and `RollerMechanism` (intakes, flywheels). Start here. |
| `common.core.fsm` | **State machines.** Subsystems with named states like NEUTRAL and INTAKE, and rules for moving between them. |
| `common.core.controllers` | `PIDFFConfig`, which holds a mechanism's gains and explains what each one does. |
| `common.hardware.motorcontroller` | TalonFX and Spark motor controllers behind one interface, plus physics models for simulation. |
| `common.hardware.camera` | PhotonVision cameras that find AprilTags and correct the robot's position. |
| `common.hardware.input` | Xbox controllers, joysticks and the team's button board. |
| `common.core.misc` | `NAR_Robot`, the base for the robot's `Robot` class. |
| `common.utility` | Console logging, Shuffleboard, NarwhalDashboard, pit system checks and SysId. |
| `common.core.swerve` | The team's older swerve code. New robots use CTRE's generated swerve instead. |

Every package has an overview in its `package-info.java`, and every class explains itself at the
top of its file. Read the code in your editor, or open `doc/index.html` for the same comments as a
website. To update `doc/` after changing comments, run `./gradlew javadoc` and copy
`build/docs/javadoc` over it.

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
repository. The notes in `build.gradle` explain how to make a release for that.

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
