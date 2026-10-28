/**
 * Motors and motor controllers.
 *
 * <ul>
 * <li>{@link common.hardware.motorcontroller.NAR_TalonFX}: a CTRE TalonFX, for Kraken and Falcon
 *     motors.</li>
 * <li>{@link common.hardware.motorcontroller.NAR_CANSpark}: a REV Spark MAX or Spark Flex, for
 *     NEO and Vortex motors.</li>
 * </ul>
 * Both are {@link common.hardware.motorcontroller.NAR_Motor}s, so the rest of the code doesn't care
 * which brand a motor is.
 *
 * <p>You'll rarely call a motor directly: create it, pass it to a mechanism in
 * {@code common.core.subsystems}, and let the mechanism configure and drive it.
 *
 * <h2>Simulation</h2>
 * The {@code sim} package has physics models for what a motor drives: a roller, something moving
 * in a line, or something pivoting. Mechanisms attach the right one for you, and
 * {@code NAR_Motor.updateSimulation(0.02)} in {@code Robot.simulationPeriodic()} steps them all.
 * The motor controllers' own simulations then report the simulated position and speed, so robot
 * code reads them exactly as it would on a real robot.
 */
package common.hardware.motorcontroller;
