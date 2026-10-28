/**
 * The team's own swerve drive code, from before 2025.
 *
 * <p>Robots now use CTRE's generated swerve code instead ({@code CommandSwerveDrivetrain} in the
 * robot project), which runs odometry faster and is kept up to date by CTRE. This package is
 * still here for older robots, and as an example of how swerve works:
 * <ul>
 * <li>{@link common.core.swerve.SwerveModule}: one wheel, with a motor that drives it, a motor
 *     that steers it, and a CANcoder that knows which way it points.</li>
 * <li>{@link common.core.swerve.SwerveBase}: four modules working together, plus a pose
 *     estimator that tracks where the robot is.</li>
 * <li>{@link common.core.swerve.CTREModuleState} and {@link common.core.swerve.SwerveConversions}:
 *     the math in between.</li>
 * </ul>
 */
package common.core.swerve;
