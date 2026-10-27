package common.hardware.motorcontroller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_Motor.StatusFrames;
import common.hardware.motorcontroller.sim.RollerSimModel;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;

class NAR_MotorTest {

    private static final double TOLERANCE = 1e-9;

    // Units used by most tests: a 1:4 reduction measured in mechanism rotations per second
    private static final double DISTANCE_FACTOR = 0.25; // mechanism rotations per motor rotation
    private static final double TIME_FACTOR = 60; // seconds instead of minutes

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    private static FakeMotor motorWithUnits() {
        final FakeMotor motor = FakeMotor.talonLike(1);
        motor.setUnitConversionFactor(DISTANCE_FACTOR);
        motor.setTimeConversionFactor(TIME_FACTOR);
        return motor;
    }

    @Test
    void positionAndVelocityAreInConfiguredUnits() {
        final FakeMotor motor = motorWithUnits();
        motor.rawPosition = 8; // motor rotations
        motor.rawVelocity = 600; // motor RPM

        assertEquals(2, motor.getPosition(), TOLERANCE); // 8 / 4 mechanism rotations
        assertEquals(2.5, motor.getVelocity(), TOLERANCE); // 10 motor rotations per second / 4
    }

    @Test
    void positionCommandIsConvertedToMotorRotations() {
        final FakeMotor motor = motorWithUnits();
        motor.set(3, Control.Position, 0.5);

        assertEquals(Control.Position, motor.lastMode);
        assertEquals(12, motor.lastPositionRotations, TOLERANCE);
        assertEquals(0.5, motor.lastFeedforward, TOLERANCE);
    }

    @Test
    void velocityCommandIsConvertedToMotorRPM() {
        final FakeMotor motor = motorWithUnits();
        motor.set(2.5, Control.Velocity);

        assertEquals(Control.Velocity, motor.lastMode);
        assertEquals(600, motor.lastVelocityRPM, TOLERANCE);
    }

    @Test
    void percentOutputIsLimitedToFullPower() {
        final FakeMotor motor = FakeMotor.talonLike(1);
        motor.set(1.5);
        assertEquals(1, motor.lastPercentOutput, TOLERANCE);
        motor.set(-3);
        assertEquals(-1, motor.lastPercentOutput, TOLERANCE);
    }

    @Test
    void voltsAreAFractionOfTwelve() {
        final FakeMotor motor = FakeMotor.talonLike(1);
        motor.setVolts(6);
        assertEquals(0.5, motor.lastPercentOutput, TOLERANCE);
    }

    @Test
    void repeatedCommandIsOnlySentOnce() {
        final FakeMotor motor = FakeMotor.talonLike(1);
        motor.set(0.3);
        motor.set(0.3);
        assertEquals(1, motor.commandCount);
        motor.set(0.4);
        assertEquals(2, motor.commandCount);
    }

    @Test
    void continuousInputTakesTheShortWayAround() {
        final FakeMotor motor = FakeMotor.talonLike(1);
        motor.setUnitConversionFactor(360); // degrees
        motor.enableContinuousInput(-180, 180);
        motor.rawPosition = 170.0 / 360; // at 170 degrees

        // -170 degrees is 20 degrees away going up through 180, not 340 degrees going down
        motor.set(-170, Control.Position);
        assertEquals(190.0 / 360, motor.lastPositionRotations, TOLERANCE);
    }

    @Test
    void resetPositionIsInConfiguredUnits() {
        final FakeMotor motor = motorWithUnits();
        motor.resetPosition(1.5);
        assertEquals(6, motor.rawPosition, TOLERANCE);
        assertEquals(1.5, motor.getPosition(), TOLERANCE);
    }

    @Test
    void configMotorAppliesEverySetting() {
        final FakeMotor motor = FakeMotor.talonLike(1);
        motor.configMotor(new MotorConfig(DISTANCE_FACTOR, TIME_FACTOR, 40, 60, 11, true, Neutral.COAST, StatusFrames.VELOCITY));

        assertEquals(DISTANCE_FACTOR, motor.unitConversionFactor, TOLERANCE);
        assertEquals(TIME_FACTOR, motor.timeConversionFactor, TOLERANCE);
        assertEquals(40, motor.statorLimit);
        assertEquals(60, motor.supplyLimit);
        assertEquals(11, motor.voltageCompensation, TOLERANCE);
        assertTrue(motor.inverted);
        assertEquals(Neutral.COAST, motor.neutralMode);
        assertEquals(StatusFrames.VELOCITY, motor.statusFrames);
    }

    @Test
    void motorConfigCopiesFlipInversionAndSetFollowerFrames() {
        final MotorConfig config = new MotorConfig(1, 60, 40, false, Neutral.BRAKE);

        assertTrue(config.invert().inverted);
        assertEquals(StatusFrames.FOLLOWER, config.follower().statusFrames);
        assertEquals(false, config.follower().inverted);
        assertTrue(config.invertFollower().inverted);
        assertEquals(StatusFrames.FOLLOWER, config.invertFollower().statusFrames);
    }

    @Test
    void motorConfigDescribesEverySetting() {
        final String description = new MotorConfig(0.5, 60, 40, false, Neutral.BRAKE).toString();
        assertTrue(description.contains("Distance Factor: 0.5"));
        assertTrue(description.contains("Stator Limit: 40"));
        assertTrue(description.contains("Neutral Mode: BRAKE"));
    }

    @Test
    void talonPositionGainsScaleByDistanceFactor() {
        final FakeMotor motor = motorWithUnits();
        // Volts per mechanism rotation of error; kS and kG are already volts
        motor.configClosedLoop(new PIDFFConfig(8, 2, 0.4, 0.1, 0, 0, 0.3), Control.Position);

        // One motor rotation is a quarter of a mechanism rotation, so a quarter of the volts
        assertEquals(2, motor.nativeKP, TOLERANCE);
        assertEquals(0.5, motor.nativeKI, TOLERANCE);
        assertEquals(0.1, motor.nativeKD, TOLERANCE);
        assertEquals(0.1, motor.nativeKS, TOLERANCE);
        assertEquals(0.3, motor.nativeKG, TOLERANCE);
        assertEquals(0, motor.armRatio, TOLERANCE);
    }

    @Test
    void talonVelocityGainsAreConvertedToMotorRotationsPerSecond() {
        final FakeMotor motor = motorWithUnits();
        // Volts per mechanism rotation per second
        motor.configClosedLoop(new PIDFFConfig(0.4, 0, 0, 0, 0.48, 0, 0), Control.Velocity);

        // One motor rotation per second is a quarter of a mechanism rotation per second
        assertEquals(0.1, motor.nativeKP, TOLERANCE);
        assertEquals(0.12, motor.nativeKV, TOLERANCE);
    }

    @Test
    void sparkGainsAreAFractionOfTwelveVoltsPerRPMWithAOneMillisecondLoop() {
        final FakeMotor motor = FakeMotor.sparkLike(1);
        motor.setUnitConversionFactor(DISTANCE_FACTOR);
        motor.setTimeConversionFactor(TIME_FACTOR);
        motor.configClosedLoop(new PIDFFConfig(24, 24, 24, 0.2, 2.4, 0, 0), Control.Velocity);

        // One motor RPM is 1/240 of a mechanism rotation per second
        final double scale = 1.0 / 240 / 12;
        assertEquals(24 * scale, motor.nativeKP, TOLERANCE);
        assertEquals(24 * scale * 0.001, motor.nativeKI, TOLERANCE);
        assertEquals(24 * scale / 0.001, motor.nativeKD, TOLERANCE);
        // Spark feedforward stays in volts
        assertEquals(0.2, motor.nativeKS, TOLERANCE);
        assertEquals(2.4 / 240, motor.nativeKV, TOLERANCE);
    }

    @Test
    void talonArmGainsAreInArmRotations() {
        final double gearing = 50; // motor rotations per arm rotation
        final FakeMotor motor = FakeMotor.talonLike(1);
        motor.setUnitConversionFactor(360 / gearing); // degrees of arm per motor rotation
        motor.configArmClosedLoop(new PIDFFConfig(0.1, 0, 0, 0, 0, 0, 0.5), gearing);

        assertEquals(gearing, motor.mechanismRatio, TOLERANCE);
        assertEquals(gearing, motor.armRatio, TOLERANCE);
        // 0.1 volts per degree is 36 volts per arm rotation
        assertEquals(36, motor.nativeKP, TOLERANCE);
        assertEquals(0.5, motor.nativeKG, TOLERANCE);
    }

    @Test
    void nonArmClosedLoopResetsTheMechanismRatio() {
        final FakeMotor motor = FakeMotor.talonLike(1);
        motor.configArmClosedLoop(new PIDFFConfig(1), 50);
        motor.configClosedLoop(new PIDFFConfig(1), Control.Position);
        assertEquals(1, motor.mechanismRatio, TOLERANCE);
    }

    @Test
    void simulationStepMovesTheEncoder() {
        final FakeMotor motor = FakeMotor.talonLike(1);
        motor.enableSim(new RollerSimModel(DCMotor.getKrakenX60(1), 1, 0.001));
        motor.setVolts(12);
        for (int i = 0; i < 50; i++) {
            NAR_Motor.updateSimulation(0.02);
        }
        assertTrue(motor.getVelocity() > 0);
        assertTrue(motor.getPosition() > 0);
    }
}
