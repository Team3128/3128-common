package common.hardware.motorcontroller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class NAR_SimMotorTest {

    private static final double MOI = 0.001; // kg*m^2
    private static final double PERIOD = 0.02; // seconds per robot loop

    private NAR_SimMotor motor;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        motor = new NAR_SimMotor(1, DCMotor.getKrakenX60(1), 1, MOI);
    }

    /** Advances simulated time, reading the motor each loop like a periodic() would. */
    private void run(double seconds) {
        for (int i = 0; i < seconds / PERIOD; i++) {
            SimHooks.stepTiming(PERIOD);
            motor.getPosition();
        }
    }

    @Test
    void startsAtRest() {
        assertEquals(0, motor.getPosition(), 1e-9);
        assertEquals(0, motor.getVelocity(), 1e-9);
    }

    @Test
    void positiveVoltageSpinsPositive() {
        motor.setVolts(12);
        run(1);
        assertTrue(motor.getPosition() > 0);
        // Kraken X60 free speed is about 6000 RPM, reached within 1 s
        assertEquals(6000, motor.getVelocity(), 600);
    }

    @Test
    void invertedMotorStillReadsPositiveWhenCommandedPositive() {
        motor.setInverted(true);
        motor.setVolts(12);
        run(1);
        assertTrue(motor.getPosition() > 0);
    }

    @Test
    void stopsWhenVoltageRemoved() {
        motor.setVolts(12);
        run(1);
        motor.setVolts(0);
        run(2);
        assertEquals(0, motor.getVelocity(), 5);
    }

    @Test
    void resetPositionSetsReadingWithoutMoving() {
        motor.setVolts(12);
        run(0.5);
        motor.set(0);
        motor.resetPosition(0);
        assertEquals(0, motor.getPosition(), 1e-6);
    }

    @Test
    void positionControlReachesTarget() {
        motor.configPID(new common.core.controllers.PIDFFConfig(0.5, 0, 0.01));
        motor.set(5, NAR_Motor.Control.Position);
        run(3);
        assertEquals(5, motor.getPosition(), 0.05);
    }

    @Test
    void solidCylinderMoiMatchesFormula() {
        assertEquals(0.5 * 2.0 * 0.1 * 0.1, NAR_SimMotor.solidCylinderMoi(2.0, 0.1), 1e-12);
    }

    @Test
    void linearMoiMatchesFormula() {
        // 2 kg moving 2*pi meters per rotation is 1 meter per radian, so J = m
        assertEquals(2.0, NAR_SimMotor.linearMoi(2.0, 2 * Math.PI), 1e-12);
    }
}
