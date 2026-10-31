# Motors and motor controllers

| Class | Hardware |
|---|---|
| [`NAR_TalonFX`](NAR_TalonFX.java) | CTRE TalonFX, for Kraken and Falcon motors |
| [`NAR_CANSpark`](NAR_CANSpark.java) | REV Spark MAX and Spark Flex, for NEO and Vortex motors |

Both extend [`NAR_Motor`](NAR_Motor.java), so the rest of the code does not depend on the brand.

Robot code rarely calls a motor directly. Create it, pass it to a mechanism in
[`common.core.subsystems`](../../core/subsystems/README.md), and let the mechanism configure and
drive it.

## Units

A motor's encoder counts rotations of the motor. `MotorConfig` converts those into units suited to
the mechanism, such as meters or degrees, so `getPosition()`, `getVelocity()` and `set(...)` all use
those units.

## Settings

Settings such as current limits come in two versions. `setStatorLimit` sends the change to the motor
controller immediately. `setStatorLimitNoApply` only stores it, so that `configMotor` can send every
setting at once with `apply()`.

## Repeated commands

`set(value, mode, feedForward)` skips a command identical to the last one, so calling it every loop
with the same setpoint does not flood the CAN bus.

## Simulation

The [`sim`](sim) package has physics models for what a motor drives: a roller, something moving in a
line, or something pivoting. Mechanisms attach the right model automatically, and
`NAR_Motor.updateSimulation(0.02)` in `Robot.simulationPeriodic()` steps them all. The motor
controllers' own simulations then report the simulated position and speed, so robot code reads them
exactly as it would on a real robot.
