package common.core.subsystems;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.MockMotor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * Behavior every mechanism shares, checked through a {@link LinearMechanism} (control loop on
 * the motor controller) and a plain {@link MechanismBase} (control loop on the roboRIO).
 */
class MechanismBaseTest {

    private static final double TOLERANCE = 1e-9;
    private static final DCMotor KRAKEN = DCMotor.getKrakenX60(1);

    // A slider measured in meters, moving 1 cm per motor rotation
    private static final double METERS_PER_ROTATION = 0.01;
    private static final MotorConfig CONFIG = new MotorConfig(METERS_PER_ROTATION, 60, 40, false, Neutral.BRAKE);
    private static final double SLIDER_TOLERANCE = 0.005;
    private static final double MIN = 0, MAX = 0.3;

    private static int nextName = 0;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void restoreTime() {
        SimHooks.resumeTiming();
    }

    /** Each mechanism gets its own Shuffleboard tab, so tests don't share widgets. */
    private static String uniqueName() {
        return "MechanismBaseTest" + nextName++;
    }

    private static LinearMechanism slider(PIDFFConfig gains, MockMotor... motors) {
        return new LinearMechanism(uniqueName(), CONFIG, gains, SLIDER_TOLERANCE, MIN, MAX, 1, 0, 2, KRAKEN, motors);
    }

    private static LinearMechanism slider(MockMotor... motors) {
        return slider(new PIDFFConfig(20), motors);
    }

    @Test
    void setSetpointEnablesAndSendsThePositionToTheLeader() {
        final MockMotor leader = MockMotor.talonLike(1);
        final LinearMechanism slider = slider(leader);

        slider.setSetpoint(0.2);
        slider.periodic();

        assertTrue(slider.isEnabled());
        assertEquals(Control.Position, leader.lastMode);
        assertEquals(0.2 / METERS_PER_ROTATION, leader.lastPositionRotations, TOLERANCE);
    }

    @Test
    void setpointIsLimitedToTheInputRange() {
        final LinearMechanism slider = slider(MockMotor.talonLike(1));
        slider.setSetpoint(1);
        assertEquals(MAX, slider.getSetpoint(), TOLERANCE);
        slider.setSetpoint(-1);
        assertEquals(MIN, slider.getSetpoint(), TOLERANCE);
    }

    @Test
    void atSetpointUsesTheTolerance() {
        final MockMotor leader = MockMotor.talonLike(1);
        final LinearMechanism slider = slider(leader);
        slider.setSetpoint(0.2);

        leader.rawPosition = 0.203 / METERS_PER_ROTATION;
        assertTrue(slider.atSetpoint());

        leader.rawPosition = 0.21 / METERS_PER_ROTATION;
        assertFalse(slider.atSetpoint());
    }

    @Test
    void positionMechanismKeepsHoldingOnceAtSetpoint() {
        final MockMotor leader = MockMotor.talonLike(1);
        final LinearMechanism slider = slider(leader);
        slider.setSetpoint(0.2);
        leader.rawPosition = 0.2 / METERS_PER_ROTATION;

        slider.periodic();

        assertTrue(slider.isEnabled());
    }

    @Test
    void disableStopsTheMotors() {
        final MockMotor leader = MockMotor.talonLike(1);
        final LinearMechanism slider = slider(leader);
        slider.setSetpoint(0.2);
        slider.periodic();

        slider.disable();

        assertFalse(slider.isEnabled());
        assertEquals(Control.PercentOutput, leader.lastMode);
        assertEquals(0, leader.lastPercentOutput, TOLERANCE);
    }

    @Test
    void openLoopCommandsTurnOffClosedLoop() {
        final MockMotor leader = MockMotor.talonLike(1);
        final LinearMechanism slider = slider(leader);
        slider.setSetpoint(0.2);

        slider.run(0.5);
        assertFalse(slider.isEnabled());
        assertEquals(0.5, leader.lastPercentOutput, TOLERANCE);

        slider.runVolts(-6);
        assertEquals(-0.5, leader.lastPercentOutput, TOLERANCE);
    }

    @Test
    void stopDisablesAndStopsTheMotors() {
        final MockMotor leader = MockMotor.talonLike(1);
        final LinearMechanism slider = slider(leader);
        slider.run(0.5);

        slider.stop();

        assertFalse(slider.isEnabled());
        assertEquals(0, leader.lastPercentOutput, TOLERANCE);
    }

    @Test
    void resetMovesEveryEncoderToTheResetPosition() {
        final MockMotor leader = MockMotor.talonLike(1);
        final MockMotor follower = MockMotor.talonLike(2);
        final LinearMechanism slider = slider(leader, follower);
        leader.rawPosition = 5;
        follower.rawPosition = 7;

        slider.reset();

        assertEquals(MIN, slider.getPosition(), TOLERANCE);
        assertEquals(MIN, follower.getPosition(), TOLERANCE);
    }

    @Test
    void extraMotorsFollowTheLeaderOnTheMotorController() {
        final MockMotor leader = MockMotor.talonLike(1);
        final MockMotor follower = MockMotor.talonLike(2);
        final LinearMechanism slider = slider(leader, follower);

        slider.setSetpoint(0.2);
        slider.periodic();
        slider.run(0.5);

        assertSame(leader, follower.leader);
        assertFalse(follower.opposesLeader);
        // Only the leader is commanded or runs the control loop
        assertNull(follower.lastMode);
        assertEquals(0, follower.closedLoopConfigCount);
        assertEquals(1, leader.closedLoopConfigCount);
    }

    @Test
    void invertingAFollowerMakesItOpposeTheLeader() {
        final MockMotor follower = MockMotor.talonLike(2);
        final LinearMechanism slider = slider(MockMotor.talonLike(1), follower);

        slider.invertMotor(1);

        assertTrue(follower.opposesLeader);
        // A follower's own inversion isn't used, so it's left alone
        assertFalse(follower.inverted);
    }

    @Test
    void invertingTheLeaderInvertsItsOwnOutput() {
        final MockMotor leader = MockMotor.talonLike(1);
        final LinearMechanism slider = slider(leader);

        slider.invertMotor(0);

        assertTrue(leader.inverted);
    }

    @Test
    void motorThatCannotFollowIsCommandedDirectly() {
        final MockMotor leader = MockMotor.talonLike(1);
        final MockMotor other = MockMotor.cannotFollow(2);
        final LinearMechanism slider = slider(leader, other);

        slider.setSetpoint(0.2);
        slider.periodic();

        assertEquals(1, other.closedLoopConfigCount);
        assertEquals(Control.Position, other.lastMode);
        assertEquals(0.2 / METERS_PER_ROTATION, other.lastPositionRotations, TOLERANCE);
    }

    @Test
    void applyGainsSendsChangedGainsToTheMotorController() {
        final PIDFFConfig gains = new PIDFFConfig(20);
        final MockMotor leader = MockMotor.talonLike(1);
        final LinearMechanism slider = slider(gains, leader);
        assertEquals(20 * METERS_PER_ROTATION, leader.nativeKP, TOLERANCE);

        gains.setkP(40);
        slider.applyGains();

        assertEquals(40 * METERS_PER_ROTATION, leader.nativeKP, TOLERANCE);
    }

    @Test
    void safetyTimeoutDisablesAMechanismThatNeverGetsThere() {
        SimHooks.pauseTiming();
        final LinearMechanism slider = slider(MockMotor.talonLike(1)); // the encoder never moves
        slider.setSetpoint(0.2);
        slider.periodic();
        assertTrue(slider.isEnabled());

        SimHooks.stepTiming(5.1);
        slider.periodic();

        assertFalse(slider.isEnabled());
    }

    @Test
    void reachingTheSetpointRestartsTheSafetyTimer() {
        SimHooks.pauseTiming();
        final MockMotor leader = MockMotor.talonLike(1);
        final LinearMechanism slider = slider(leader);
        slider.setSetpoint(0.2);
        leader.rawPosition = 0.2 / METERS_PER_ROTATION;

        for (int i = 0; i < 400; i++) { // 8 seconds of holding
            SimHooks.stepTiming(0.02);
            slider.periodic();
        }

        assertTrue(slider.isEnabled());
    }

    @Test
    void stateIsLoggedEveryLoop() {
        final MockMotor leader = MockMotor.talonLike(1);
        final MockMotor follower = MockMotor.talonLike(2);
        final LinearMechanism slider = slider(leader, follower);
        final NetworkTable logged = NetworkTableInstance.getDefault().getTable("Mechanisms").getSubTable(slider.getName());

        slider.setSetpoint(0.2);
        leader.rawPosition = 0.1 / METERS_PER_ROTATION;
        slider.periodic();

        assertEquals(0.2, logged.getEntry("Setpoint").getDouble(-1), TOLERANCE);
        assertEquals(0.1, logged.getEntry("Measurement").getDouble(-1), TOLERANCE);
        assertEquals(0.1, logged.getEntry("Position").getDouble(-1), TOLERANCE);
        assertTrue(logged.getEntry("Enabled").getBoolean(false));
        assertFalse(logged.getEntry("AtSetpoint").getBoolean(true));
        // One value per motor
        assertArrayEquals(new double[] {25, 25}, logged.getEntry("TemperatureCelsius").getDoubleArray(new double[0]), TOLERANCE);
    }

    @Test
    void sliderPoseMovesAlongItsIncline() {
        final MockMotor motor = MockMotor.talonLike(1);
        final double min = 0.1;
        final LinearMechanism slider = new LinearMechanism(uniqueName(), CONFIG, new PIDFFConfig(20), SLIDER_TOLERANCE,
            min, MAX, 1, Math.toRadians(30), 2, KRAKEN, motor);

        motor.rawPosition = (min + 0.2) / METERS_PER_ROTATION; // 0.2 m out from where it starts
        final Pose3d pose = slider.getComponentPose();

        assertEquals(0.2 * Math.cos(Math.toRadians(30)), pose.getX(), 1e-6);
        assertEquals(0, pose.getY(), 1e-6);
        assertEquals(0.2 * Math.sin(Math.toRadians(30)), pose.getZ(), 1e-6);
    }

    @Test
    void openLoopMechanismCannotTakeASetpoint() {
        final RollerMechanism roller = new RollerMechanism(uniqueName(), CONFIG, KRAKEN, 1, 0.001, MockMotor.talonLike(1));
        assertThrows(NullPointerException.class, () -> roller.setSetpoint(1));
    }

    /** A mechanism whose control loop runs on the roboRIO, with a P controller and 1 V of feedforward. */
    private static class RioMechanism extends MechanismBase {
        static final double KP = 2, FEEDFORWARD_VOLTS = 1;
        final AtomicInteger resets;

        RioMechanism(MockMotor motor, AtomicInteger resets) {
            super(new PIDFFConfig(), (measurement, setpoint) -> KP * (setpoint - measurement),
                measurement -> resets.incrementAndGet(), motor::getPosition, 0.1, 0, CONFIG, motor);
            setName(uniqueName());
            this.resets = resets;
        }

        @Override
        protected double calculateFeedforward(double pidOutput) {
            return FEEDFORWARD_VOLTS;
        }
    }

    @Test
    void roboRioLoopAppliesFeedbackPlusFeedforwardAsVolts() {
        final MockMotor motor = MockMotor.talonLike(1);
        final RioMechanism mechanism = new RioMechanism(motor, new AtomicInteger());

        mechanism.setSetpoint(3);
        mechanism.periodic();

        final double volts = RioMechanism.KP * 3 + RioMechanism.FEEDFORWARD_VOLTS;
        assertEquals(Control.PercentOutput, motor.lastMode);
        assertEquals(volts, motor.lastVolts(), 1e-6);
    }

    @Test
    void roboRioLoopIsLimitedToTwelveVolts() {
        final MockMotor motor = MockMotor.talonLike(1);
        final RioMechanism mechanism = new RioMechanism(motor, new AtomicInteger());

        mechanism.setSetpoint(100);
        mechanism.periodic();

        assertEquals(MechanismBase.MAX_VOLTS, motor.lastVolts(), 1e-6);
    }

    @Test
    void roboRioControllerIsResetOnlyWhenEnabledFromDisabled() {
        final AtomicInteger resets = new AtomicInteger();
        final RioMechanism mechanism = new RioMechanism(MockMotor.talonLike(1), resets);

        mechanism.setSetpoint(1);
        mechanism.setSetpoint(2);
        assertEquals(1, resets.get());

        mechanism.disable();
        mechanism.setSetpoint(1);
        assertEquals(2, resets.get());
    }
}
