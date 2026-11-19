# Mechanisms

A mechanism is a part of the robot that motors move, such as an elevator, arm or roller.

## Choosing a class

| Class | Motion | Used for |
|---|---|---|
| [`LinearMechanism`](LinearMechanism.java) | Moves in a straight line to a position | Elevators, sliders, telescopes |
| [`PivotMechanism`](PivotMechanism.java) | Rotates to an angle | Arms, wrists, shooter hoods, turrets |
| [`RollerMechanism`](RollerMechanism.java) | Spins, either on and off or at a set speed | Intakes, hoppers, kickers, shooter flywheels |

All three extend [`MechanismBase`](MechanismBase.java), which provides setpoints, open-loop control,
resetting, logging and Shuffleboard tuning.

## Basic use

| Method | Purpose |
|---|---|
| `setSetpoint(value)` | Moves to a position or speed and holds it |
| `run(power)`, `runVolts(volts)` | Drives the motors directly, without a setpoint |
| `stop()` | Stops the motors |
| `reset()` | Tells the mechanism it is at its starting position |

## Units

A mechanism's units are set by its `MotorConfig`: meters for an elevator, degrees for an arm,
rotations per second for a flywheel. Setpoints, measurements, tolerances and gains all use those
units, and conversion to the motor's own units is handled for you.

## Starting position

Motor encoders do not know where a mechanism is when the robot turns on. Each mechanism assumes it
starts at a known position, usually against a hard stop. If the position slips, push the mechanism
back against the stop and call `reset()`.

## Where the control loop runs

A control loop measures the mechanism, compares the measurement with the setpoint, and calculates
the volts to send. The gains are described in [`common.core.controllers`](../controllers/README.md).
The loop can run in one of two places.

**On the motor controller.** This is the usual choice. The TalonFX or Spark runs PID and feedforward
1000 times a second on its own encoder, 50 times faster than robot code. Gains are given in the
mechanism's units and converted for the motor controller.

```java
super(gains, Control.Position, TOLERANCE, HOME, MOTOR_CONFIG, motor);
```

**On the roboRIO.** Use this for a WPILib controller such as `BangBangController`, or when the
measurement does not come from the motor's encoder, such as a gyro or camera. The controller
calculates volts from `(measurement, setpoint)`.

```java
super(gains, pid::calculate, m -> pid.reset(), gyro::getAngle, TOLERANCE, HOME, MOTOR_CONFIG, motor);
```

Anything the motor controller cannot calculate itself, such as gravity on a wrist that depends on
the arm below it, goes in `calculateFeedforward`. It runs on the roboRIO and is sent with each
setpoint.

## Multiple motors

When several motors drive one mechanism, the first is the leader and the rest follow it, on the
motor controller where possible. Only the leader runs the control loop.

## Mounting mechanisms on each other

`setParent(...)` mounts one mechanism on another, such as a wrist on an arm or an arm on an
elevator. Their 3D poses then move together in AdvantageScope, and gravity is calculated from the
whole stack: a wrist's gravity uses the arm's angle plus its own. A mechanism's mass should include
everything mounted on it.

## Logging

Every loop, the setpoint, measurement, volts, and each motor's current and temperature are logged
under `Mechanisms/<name>` in NetworkTables for AdvantageScope. `getComponentPose()` gives the
position of the moving part for AdvantageScope's 3D view.

## Simulation

Each mechanism gives its motor a physics model, so it moves realistically in simulation with no
changes to robot code. The model uses the mass, length and motor type passed to the constructor.
