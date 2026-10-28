/**
 * Mechanisms: the parts of a robot that motors move, like elevators, arms and rollers.
 *
 * <h2>Pick the one that matches how it moves</h2>
 * <ul>
 * <li>{@link common.core.subsystems.LinearMechanism}: moves in a straight line to a position.
 *     Elevators, sliders, telescopes.</li>
 * <li>{@link common.core.subsystems.PivotMechanism}: rotates to an angle. Arms, wrists, shooter
 *     hoods, turrets.</li>
 * <li>{@link common.core.subsystems.RollerMechanism}: spins, either just on and off or at a set
 *     speed. Intakes, hoppers, kickers, shooter flywheels.</li>
 * </ul>
 * All three are built on {@link common.core.subsystems.MechanismBase}, which has everything they
 * share: setpoints, running open loop, resetting, logging and Shuffleboard tuning.
 *
 * <h2>Units</h2>
 * You pick a mechanism's units with its {@code MotorConfig}: meters for an elevator, degrees for
 * an arm, rotations per second for a flywheel. Setpoints, measurements, tolerances and gains are
 * then all in those units, and converting to the motor's own units is done for you.
 *
 * <h2>Gains</h2>
 * Every mechanism with a setpoint takes a {@link common.core.controllers.PIDFFConfig}, which
 * explains what each gain does. The control loop runs on the motor controller, 1000 times a
 * second, so even fast mechanisms stay smooth.
 *
 * <h2>Starting position</h2>
 * Motor encoders don't know where a mechanism is when the robot turns on. Each mechanism assumes
 * it starts at a known position, usually against a hard stop. If something slips, push it back
 * against the stop and call {@code reset()}.
 *
 * <h2>Mechanisms on mechanisms</h2>
 * {@code setParent(...)} mounts one mechanism on another, like a wrist on an arm or an arm on an
 * elevator. Their 3D poses then move together in AdvantageScope, and gravity is worked out from
 * the whole stack, e.g. a wrist's from the arm's angle plus its own. A mechanism's mass should
 * include everything mounted on it.
 *
 * <h2>Simulation</h2>
 * Each mechanism gives its motor a physics model, so it moves realistically in simulation with no
 * changes to robot code. The mass, length and motor type passed to the constructor are what the
 * model uses.
 */
package common.core.subsystems;
