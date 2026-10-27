package common.core.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class CTREModuleStateTest {

    private static final double TOLERANCE = 1e-9;
    private static final double SPEED = 2; // meters per second

    private static SwerveModuleState optimize(double desiredDegrees, double currentDegrees) {
        return CTREModuleState.optimize(
            new SwerveModuleState(SPEED, Rotation2d.fromDegrees(desiredDegrees)), Rotation2d.fromDegrees(currentDegrees));
    }

    @Test
    void smallTurnKeepsTheDirection() {
        final SwerveModuleState state = optimize(30, 0);
        assertEquals(30, state.angle.getDegrees(), TOLERANCE);
        assertEquals(SPEED, state.speedMetersPerSecond, TOLERANCE);
    }

    @Test
    void turnOfMoreThan90DegreesReversesTheWheelInstead() {
        final SwerveModuleState state = optimize(180, 0);
        assertEquals(0, state.angle.getDegrees(), TOLERANCE);
        assertEquals(-SPEED, state.speedMetersPerSecond, TOLERANCE);
    }

    @Test
    void negativeTurnOfMoreThan90DegreesAlsoReverses() {
        final SwerveModuleState state = optimize(-135, 0);
        assertEquals(45, state.angle.getDegrees(), TOLERANCE);
        assertEquals(-SPEED, state.speedMetersPerSecond, TOLERANCE);
    }

    @Test
    void angleStaysNearTheCurrentAngleAfterManyTurns() {
        // The module has turned past a full rotation; the motor controller needs a nearby angle, not 20
        final SwerveModuleState state = optimize(20, 370);
        assertEquals(380, state.angle.getDegrees(), TOLERANCE);
        assertEquals(SPEED, state.speedMetersPerSecond, TOLERANCE);
    }
}
