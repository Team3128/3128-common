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
    void hollowCylinderMoiMatchesFormula() {
        assertEquals(0.5 * 2.0 * (0.04 * 0.04 + 0.05 * 0.05), RollerSimModel.hollowCylinderMoi(2.0, 0.04, 0.05), 1e-12);
    }

    @Test
    void hollowCylinderWithNoHoleIsSolid() {
        assertEquals(RollerSimModel.solidCylinderMoi(2.0, 0.05), RollerSimModel.hollowCylinderMoi(2.0, 0, 0.05), 1e-12);
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
        final LinearSimModel slider = new LinearSimModel(KRAKEN, rotationsPerMeter, 2, 0, 0.3, 0);
        run(slider, 12, 3);
        assertEquals(0.3 * rotationsPerMeter, slider.getRotorRotations(), 1e-6);
    }

    @Test
    void linearWithGravityFallsWithoutPower() {
        final LinearSimModel elevator = new LinearSimModel(KRAKEN, 50, 5, 0, 1, Math.PI / 2);
        run(elevator, 12, 0.5);
        final double raised = elevator.getRotorRotations();
        run(elevator, 0, 2);
        assertTrue(elevator.getRotorRotations() < raised);
    }

    @Test
    void horizontalSliderStaysPutWithoutPower() {
        final LinearSimModel slider = new LinearSimModel(KRAKEN, 50, 2, 0, 0.3, 0);
        run(slider, 6, 0.2);
        run(slider, 0, 2); // let it coast to a stop
        final double stopped = slider.getRotorRotations();
        run(slider, 0, 1);
        assertEquals(stopped, slider.getRotorRotations(), 1e-3);
    }

    @Test
    void steeperInclineRisesSlower() {
        final LinearSimModel shallow = new LinearSimModel(KRAKEN, 50, 5, 0, 1, Math.toRadians(20));
        final LinearSimModel steep = new LinearSimModel(KRAKEN, 50, 5, 0, 1, Math.toRadians(80));
        run(shallow, 12, 0.3);
        run(steep, 12, 0.3);
        assertTrue(steep.getRotorRotations() < shallow.getRotorRotations());
    }

    @Test
    void gearboxOfTwoHasTwiceTheStallTorque() {
        assertEquals(2 * KRAKEN.stallTorqueNewtonMeters, MotorSimModel.gearbox(KRAKEN, 2).stallTorqueNewtonMeters, 1e-9);
        assertEquals(KRAKEN.freeSpeedRadPerSec, MotorSimModel.gearbox(KRAKEN, 2).freeSpeedRadPerSec, 1e-9);
    }

    @Test
    void armStartsAtZeroRotorRotationsWhereverItStarts() {
        final PivotSimModel arm = new PivotSimModel(KRAKEN, 50, 0.5, 3, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, true);
        assertEquals(0, arm.getRotorRotations(), 1e-9);
    }

    @Test
    void onlyAPivotWithGravityFallsWithoutPower() {
        // Both start level
        final PivotSimModel arm = new PivotSimModel(KRAKEN, 50, 0.5, 3, -Math.PI / 2, Math.PI / 2, 0, true);
        final PivotSimModel turret = new PivotSimModel(KRAKEN, 50, 0.5, 3, -Math.PI, Math.PI, 0, false);

        run(arm, 0, 0.5);
        run(turret, 0, 0.5);

        assertTrue(arm.getRotorRotations() < -1);
        assertEquals(0, turret.getRotorRotations(), 1e-9);
    }

    @Test
    void armRisesWithPositiveVoltage() {
        final PivotSimModel arm = new PivotSimModel(KRAKEN, 50, 0.5, 3, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, true);
        run(arm, 6, 0.5);
        assertTrue(arm.getRotorRotations() > 0);
    }
}
