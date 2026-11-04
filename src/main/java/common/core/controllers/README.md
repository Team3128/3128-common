# Gains

[`PIDFFConfig`](PIDFFConfig.java) holds the tuning numbers for a mechanism's control loop. Every gain
produces volts for the motor. Units are whatever the mechanism measures in, such as meters for an
elevator or degrees for an arm.

## Feedback (PID)

Feedback responds to the error: how far the mechanism is from its setpoint.

| Gain | Unit | Effect |
|---|---|---|
| kP | Volts per unit of error | The main gain. Larger values push harder; too large overshoots and oscillates. |
| kI | Volts per unit of error, per second | Slowly builds up to remove a small persistent error. Usually 0. |
| kD | Volts per unit/second of change in error | Brakes as the mechanism approaches the setpoint, reducing overshoot. Often 0. |

## Feedforward

Feedforward applies what the mechanism is expected to need, before any error appears.

| Gain | Unit | Effect |
|---|---|---|
| kS | Volts | Overcomes friction to start the mechanism moving. |
| kV | Volts per unit/second | Maintains a speed. The main gain for a flywheel. |
| kA | Volts per unit/second^2 | Accelerates the mechanism. Not used by the motor controllers. |
| kG | Volts | Holds the mechanism against gravity: constant for an elevator, multiplied by the cosine of the angle for an arm, so it is strongest when the arm is level. |

## Tuning

Good feedforward does most of the work and leaves PID only small errors to correct, which makes a
mechanism fast and smooth. Tune feedforward first, for example with SysId, then add kP.
