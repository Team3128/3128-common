package common.hardware.input;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import common.hardware.input.NAR_XboxController.XboxButton;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

class NAR_XboxControllerTest {

    private static final int PORT = 0;
    private static final double TOLERANCE = 1e-9;

    private NAR_XboxController controller;
    private XboxControllerSim sim;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void createController() {
        controller = new NAR_XboxController(PORT);
        sim = new XboxControllerSim(controller);
    }

    @Test
    void stickInsideTheDeadbandReadsZero() {
        sim.setLeftX(0.03);
        sim.setRightX(-0.04);
        sim.notifyNewData();

        assertEquals(0, controller.getLeftX(), TOLERANCE);
        assertEquals(0, controller.getRightX(), TOLERANCE);
    }

    @Test
    void stickOutsideTheDeadbandReadsItsValue() {
        sim.setLeftX(0.5);
        sim.setRightX(-0.7);
        sim.notifyNewData();

        assertEquals(0.5, controller.getLeftX(), TOLERANCE);
        assertEquals(-0.7, controller.getRightX(), TOLERANCE);
    }

    @Test
    void rightStickUpIsPositive() {
        // WPILib reports up as negative
        sim.setRightY(-0.6);
        sim.notifyNewData();
        assertEquals(0.6, controller.getRightY(), TOLERANCE);
    }

    @Test
    void deadbandCanBeChanged() {
        controller.setLeftXDeadband(0.2);
        sim.setLeftX(0.15);
        sim.notifyNewData();
        assertEquals(0, controller.getLeftX(), TOLERANCE);
    }

    @Test
    void triggerCountsAsPressedPastHalfway() {
        sim.setLeftTriggerAxis(0.4);
        sim.setRightTriggerAxis(0.6);
        sim.notifyNewData();

        assertFalse(controller.getButton(XboxButton.kLeftTrigger).getAsBoolean());
        assertTrue(controller.getButton(XboxButton.kRightTrigger).getAsBoolean());
    }

    @Test
    void buttonsReadTheirState() {
        sim.setAButton(true);
        sim.setBButton(false);
        sim.notifyNewData();

        assertTrue(controller.getButton(XboxButton.kA).getAsBoolean());
        assertFalse(controller.getButton(XboxButton.kB).getAsBoolean());
    }
}
