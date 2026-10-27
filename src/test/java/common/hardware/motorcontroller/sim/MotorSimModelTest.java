package common.hardware.motorcontroller.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.system.plant.DCMotor;

class MotorSimModelTest {

    private static final double PERIOD = 0.02; // seconds per robot loop
    private static final DCMotor KRAKEN = DCMotor.getKrakenX60(1);

    /** Applies a constant voltage for some time. */
    private static void run(MotorSimModel model, double volts, double seconds) {
        for (int i = 0; i < seconds / PERIOD; i++) {
            model.update(volts, PERIOD);
        }
    }

    @Test
    void solidCylinderMoiMatchesFormula() {
        assertEquals(0.5 * 2.0 * 0.1 * 0.1, RollerSimModel.solidCylinderMoi(2.0, 0.1), 1e-12);
    }

    @Test
    void rollerStartsAtZero() {
        final RollerSimModel roller = new RollerSimModel(KRAKEN, 3, 0.001);
        assertEquals(0, roller.getRotorRotations(), 1e-9);
        assertEquals(0, roller.getRotorRPS(), 1e-9);
    }

    @Test
    void rollerReachesFreeSpeedAtTheRotor() {
        final RollerSimModel roller = new RollerSimModel(KRAKEN, 3, 0.001);
        run(roller, 12, 2);
        // Rotor speed is the motor's free speed no matter the gearing
        assertEquals(KRAKEN.freeSpeedRadPerSec / (2 * Math.PI), roller.getRotorRPS(), 5);
    }

    @Test
    void rollerSpinsBackwardWithNegativeVoltage() {
        final RollerSimModel roller = new RollerSimModel(KRAKEN, 1, 0.001);
        run(roller, -6, 1);
        assertTrue(roller.getRotorRotations() < 0);
        assertTrue(roller.getRotorRPS() < 0);
    }

    @Test
    void linearStopsAtHardStop() {
        final double rotationsPerMeter = 50;
        final LinearSimModel slider = new LinearSimModel(KRAKEN, rotationsPerMeter, 2, 0, 0.3, false);
        run(slider, 12, 3);
        assertEquals(0.3 * rotationsPerMeter, slider.getRotorRotations(), 1e-6);
    }

    @Test
    void linearWithGravityFallsWithoutPower() {
        final LinearSimModel elevator = new LinearSimModel(KRAKEN, 50, 5, 0, 1, true);
        run(elevator, 12, 0.5);
        final double raised = elevator.getRotorRotations();
        run(elevator, 0, 2);
        assertTrue(elevator.getRotorRotations() < raised);
    }

    @Test
    void armStartsAtZeroRotorRotationsWhereverItStarts() {
        final ArmSimModel arm = new ArmSimModel(KRAKEN, 50, 0.5, 3, -Math.PI / 2, Math.PI / 2, -Math.PI / 2);
        assertEquals(0, arm.getRotorRotations(), 1e-9);
    }

    @Test
    void armRisesWithPositiveVoltage() {
        final ArmSimModel arm = new ArmSimModel(KRAKEN, 50, 0.5, 3, -Math.PI / 2, Math.PI / 2, -Math.PI / 2);
        run(arm, 6, 0.5);
        assertTrue(arm.getRotorRotations() > 0);
    }
}
