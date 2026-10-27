package common.core.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.MockMotor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;

class ArmMechanismTest {

    private static final double TOLERANCE = 1e-9;
    private static final DCMotor KRAKEN = DCMotor.getKrakenX60(1);

    // An arm on a 50:1 reduction, measured in degrees
    private static final double GEARING = 50;
    private static final double DEGREES_PER_MOTOR_ROTATION = 360 / GEARING;
    private static final double MIN = -90, MAX = 90, START = -90;

    private static final double KP = 0.1; // volts per degree
    private static final double KG = 0.5; // volts to hold the arm level

    private static int nextName = 0;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    private static ArmMechanism arm(double degreesPerEncoderRotation, MockMotor motor) {
        final MotorConfig config = new MotorConfig(degreesPerEncoderRotation, 60, 40, false, Neutral.BRAKE);
        return new ArmMechanism("ArmMechanismTest" + nextName++, config, new PIDFFConfig(KP, 0, 0, 0, 0, 0, KG),
            1, MIN, MAX, START, Math.PI / 180, 0.5, 3, KRAKEN, motor);
    }

    private static ArmMechanism arm(MockMotor motor) {
        return arm(DEGREES_PER_MOTOR_ROTATION, motor);
    }

    @Test
    void gravityFollowsTheArmsCosineInArmRotations() {
        final MockMotor motor = MockMotor.talonLike(1);
        arm(motor);

        assertEquals(GEARING, motor.armRatio, TOLERANCE);
        assertEquals(GEARING, motor.mechanismRatio, TOLERANCE);
        assertEquals(KG, motor.nativeKG, TOLERANCE);
        // 0.1 volts per degree is 36 volts per arm rotation
        assertEquals(KP * 360, motor.nativeKP, TOLERANCE);
    }

    @Test
    void encoderStartsAtTheStartPosition() {
        final MockMotor motor = MockMotor.talonLike(1);
        final ArmMechanism arm = arm(motor);
        assertEquals(START, arm.getPosition(), TOLERANCE);
    }

    @Test
    void setpointIsSentInMotorRotations() {
        final MockMotor motor = MockMotor.talonLike(1);
        final ArmMechanism arm = arm(motor);

        arm.setSetpoint(45);
        arm.periodic();

        assertEquals(Control.Position, motor.lastMode);
        assertEquals(45 / DEGREES_PER_MOTOR_ROTATION, motor.lastPositionRotations, TOLERANCE);
    }

    @Test
    void setpointIsLimitedToTheArmsTravel() {
        final ArmMechanism arm = arm(MockMotor.talonLike(1));
        arm.setSetpoint(180);
        assertEquals(MAX, arm.getSetpoint(), TOLERANCE);
        arm.setSetpoint(-180);
        assertEquals(MIN, arm.getSetpoint(), TOLERANCE);
    }

    @Test
    void armKeepsHoldingAgainstGravityAtSetpoint() {
        final MockMotor motor = MockMotor.talonLike(1);
        final ArmMechanism arm = arm(motor);
        arm.setSetpoint(START);

        arm.periodic();

        assertTrue(arm.isEnabled());
    }

    @Test
    void encoderOnTheArmShaftGivesAOneToOneGravityRatio() {
        final MockMotor motor = MockMotor.sparkLike(1);
        motor.setMotorRotationsPerSensorRotation(GEARING); // the encoder turns once per arm rotation
        arm(360, motor); // 360 degrees per encoder rotation

        assertEquals(1, motor.armRatio, TOLERANCE);
    }
}
