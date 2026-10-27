package common.core.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SwerveConversionsTest {

    private static final double TOLERANCE = 1e-9;
    private static final double GEAR_RATIO = 6.75; // motor rotations per wheel rotation
    private static final double CIRCUMFERENCE = 0.3; // meters

    @Test
    void oneWheelTurnIs360Degrees() {
        assertEquals(360, SwerveConversions.rotationsToDegrees(GEAR_RATIO, GEAR_RATIO), TOLERANCE);
        assertEquals(GEAR_RATIO, SwerveConversions.degreesToRotations(360, GEAR_RATIO), TOLERANCE);
    }

    @Test
    void oneWheelTurnIsOneCircumference() {
        assertEquals(CIRCUMFERENCE, SwerveConversions.rotationsToMeters(GEAR_RATIO, CIRCUMFERENCE, GEAR_RATIO), TOLERANCE);
        assertEquals(GEAR_RATIO, SwerveConversions.metersToRotations(CIRCUMFERENCE, CIRCUMFERENCE, GEAR_RATIO), TOLERANCE);
    }

    @Test
    void oneWheelTurnPerSecondIsOneCircumferencePerSecond() {
        final double motorRPM = GEAR_RATIO * 60;
        assertEquals(CIRCUMFERENCE, SwerveConversions.RPMToMPS(motorRPM, CIRCUMFERENCE, GEAR_RATIO), TOLERANCE);
        assertEquals(motorRPM, SwerveConversions.MPSToRPM(CIRCUMFERENCE, CIRCUMFERENCE, GEAR_RATIO), TOLERANCE);
    }

    @Test
    void conversionsUndoEachOther() {
        assertEquals(1.234, SwerveConversions.rotationsToDegrees(
            SwerveConversions.degreesToRotations(1.234, GEAR_RATIO), GEAR_RATIO), TOLERANCE);
        assertEquals(2.5, SwerveConversions.RPMToMPS(
            SwerveConversions.MPSToRPM(2.5, CIRCUMFERENCE, GEAR_RATIO), CIRCUMFERENCE, GEAR_RATIO), TOLERANCE);
    }
}
