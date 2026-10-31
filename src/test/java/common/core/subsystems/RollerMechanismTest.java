package common.core.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import common.core.controllers.PIDFFConfig;
import common.core.subsystems.RollerMechanism.VelocityControl;
import common.hardware.motorcontroller.MockMotor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;

class RollerMechanismTest {

    private static final double TOLERANCE = 1e-9;
    private static final double PERIOD = 0.02;
    private static final DCMotor KRAKEN = DCMotor.getKrakenX60(1);

    // Measured in motor rotations per second
    private static final MotorConfig CONFIG = new MotorConfig(1, 60, 40, false, Neutral.COAST);
    private static final double SPEED_TOLERANCE = 2;
    private static final double MOI = 0.01; // a heavy flywheel, so bang-bang overshoots little per loop

    private static int nextName = 0;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    private static RollerMechanism roller(VelocityControl control, PIDFFConfig gains, MockMotor motor) {
        return new RollerMechanism("RollerMechanismTest" + nextName++, CONFIG, control, gains, SPEED_TOLERANCE,
            KRAKEN, 1, MOI, motor);
    }

    @Test
    void pidSendsTheVelocityToTheMotorController() {
        final MockMotor motor = MockMotor.talonLike(1);
        final RollerMechanism roller = roller(VelocityControl.PID, new PIDFFConfig(0.2, 0, 0, 0, 0.1, 0, 0), motor);

        roller.setSetpoint(50);
        roller.periodic();

        assertEquals(Control.Velocity, motor.lastMode);
        assertEquals(50 * 60, motor.lastVelocityRPM, TOLERANCE);
        // The motor controller applies the feedforward itself, so none is sent with the setpoint
        assertEquals(0, motor.lastFeedforward, TOLERANCE);
        assertEquals(0.2, motor.nativeKP, TOLERANCE);
        assertEquals(0.1, motor.nativeKV, TOLERANCE);
    }

    @Test
    void rollerKeepsSpinningOnceAtSpeed() {
        final MockMotor motor = MockMotor.talonLike(1);
        final RollerMechanism roller = roller(VelocityControl.PID, new PIDFFConfig(0.2), motor);
        roller.setSetpoint(50);
        motor.rawVelocity = 50 * 60;

        roller.periodic();

        assertTrue(roller.isEnabled());
    }

    @Test
    void bangBangGivesFullPowerBelowTheSetpoint() {
        final MockMotor motor = MockMotor.talonLike(1);
        final RollerMechanism roller = roller(VelocityControl.BANG_BANG, new PIDFFConfig(0, 0, 0, 0, 0.1, 0, 0), motor);
        roller.setSetpoint(50);
        motor.rawVelocity = 30 * 60;

        roller.periodic();

        assertEquals(Control.PercentOutput, motor.lastMode);
        assertEquals(12, motor.lastVolts(), 1e-6);
    }

    @Test
    void bangBangGivesOnlyFeedforwardAboveTheSetpoint() {
        final MockMotor motor = MockMotor.talonLike(1);
        final RollerMechanism roller = roller(VelocityControl.BANG_BANG, new PIDFFConfig(0, 0, 0, 0, 0.1, 0, 0), motor);
        roller.setSetpoint(50);
        motor.rawVelocity = 60 * 60;

        roller.periodic();

        assertEquals(0.1 * 50, motor.lastVolts(), 1e-6);
    }

    @Test
    void bangBangSpinsASimulatedFlywheelUpToSpeed() {
        final MockMotor motor = MockMotor.talonLike(1);
        // A Kraken spins about 100 rotations per second at 12 V, so 0.1 V per rotation per second
        // is a little under what holds speed, as bang-bang needs
        final RollerMechanism roller = roller(VelocityControl.BANG_BANG, new PIDFFConfig(0, 0, 0, 0, 0.1, 0, 0), motor);
        roller.setSetpoint(50);

        for (int i = 0; i < 3 / PERIOD; i++) {
            roller.periodic();
            motor.step(PERIOD);
        }

        assertEquals(50, roller.getVelocity(), 5);
        assertTrue(roller.isEnabled());
    }

    @Test
    void openLoopRollerRunsAtTheGivenPower() {
        final MockMotor motor = MockMotor.talonLike(1);
        final RollerMechanism roller = new RollerMechanism("RollerMechanismTest" + nextName++, CONFIG, KRAKEN, 1, MOI, motor);

        roller.run(0.4);

        assertEquals(0.4, motor.lastPercentOutput, TOLERANCE);
        assertThrows(NullPointerException.class, () -> roller.setSetpoint(10));
    }

    @Test
    void openLoopRollerSpinsInSimulation() {
        final MockMotor motor = MockMotor.talonLike(1);
        final RollerMechanism roller = new RollerMechanism("RollerMechanismTest" + nextName++, CONFIG, KRAKEN, 1, MOI, motor);

        roller.runVolts(6);
        for (int i = 0; i < 1 / PERIOD; i++) {
            motor.step(PERIOD);
        }

        assertTrue(roller.getVelocity() > 0);
    }
}
