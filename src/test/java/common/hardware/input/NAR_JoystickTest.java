package common.hardware.input;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.JoystickSim;

class NAR_JoystickTest {

    private static final int PORT = 1;
    private static final double TOLERANCE = 1e-9;

    private NAR_Joystick joystick;
    private JoystickSim sim;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void createJoystick() {
        joystick = new NAR_Joystick(PORT);
        sim = new JoystickSim(PORT);
        sim.setAxisCount(5);
        sim.setButtonCount(16);
    }

    @Test
    void axesInsideTheDeadbandReadZero() {
        sim.setX(0.04);
        sim.setZ(-0.04);
        sim.notifyNewData();

        assertEquals(0, joystick.getX(), TOLERANCE);
        assertEquals(0, joystick.getZ(), TOLERANCE);
    }

    @Test
    void forwardIsPositive() {
        // WPILib reports forward as negative
        sim.setY(-0.5);
        sim.notifyNewData();
        assertEquals(0.5, joystick.getY(), TOLERANCE);
    }

    @Test
    void throttleIsLimitedToItsBounds() {
        sim.setThrottle(0); // the middle of the slider
        sim.notifyNewData();
        assertEquals(0.5, joystick.getThrottle(), TOLERANCE);

        sim.setThrottle(-1); // all the way up
        sim.notifyNewData();
        assertEquals(0.8, joystick.getThrottle(), TOLERANCE);

        sim.setThrottle(1); // all the way down
        sim.notifyNewData();
        assertEquals(0.3, joystick.getThrottle(), TOLERANCE);
    }

    @Test
    void throttleBoundsCanBeChanged() {
        joystick.setThrottleBounds(0, 1);
        sim.setThrottle(-1);
        sim.notifyNewData();
        assertEquals(1, joystick.getThrottle(), TOLERANCE);
    }

    @Test
    void buttonsAreNumberedFromOne() {
        sim.setRawButton(1, true);
        sim.notifyNewData();

        assertTrue(joystick.getButton(1).getAsBoolean());
        assertFalse(joystick.getButton(2).getAsBoolean());
    }

    @Test
    void axesOutsideTheDeadbandReadTheirValueAndDeadbandsCanChange() {
        sim.setX(0.5);
        sim.setZ(-0.5);
        sim.notifyNewData();
        assertEquals(0.5, joystick.getX(), 1e-6);
        assertEquals(-0.5, joystick.getZ(), 1e-6);

        joystick.setXDeadband(0.6);
        joystick.setYDeadband(0.6);
        joystick.setZDeadband(0.6);
        sim.setY(-0.5);
        sim.notifyNewData();
        assertEquals(0, joystick.getX(), 1e-6);
        assertEquals(0, joystick.getY(), 1e-6);
        assertEquals(0, joystick.getZ(), 1e-6);
    }

    @Test
    void hatSwitchDirectionsAreButtons() {
        sim.setPOVCount(1);
        sim.setPOV(90);
        sim.notifyNewData();

        assertTrue(joystick.getRightPOVButton().getAsBoolean());
        assertFalse(joystick.getUpPOVButton().getAsBoolean());
        assertFalse(joystick.getDownPOVButton().getAsBoolean());
        assertFalse(joystick.getLeftPOVButton().getAsBoolean());
    }
}
