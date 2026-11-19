package common.hardware.input;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;

class NAR_ButtonBoardTest {

    private static final int PORT = 2;

    private NAR_ButtonBoard board;
    private GenericHIDSim sim;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void createBoard() {
        board = new NAR_ButtonBoard(PORT);
        sim = new GenericHIDSim(PORT);
        sim.setButtonCount(12);
        sim.setAxisCount(2);
    }

    @Test
    void buttonsAreNumberedFromOne() {
        sim.setRawButton(3, true);
        sim.notifyNewData();

        assertTrue(board.getButton(3).getAsBoolean());
        assertFalse(board.getButton(4).getAsBoolean());
    }

    @Test
    void axisDirectionsAreButtonsThirteenToSixteen() {
        sim.setRawAxis(0, -1.0);
        sim.setRawAxis(1, 1.0);
        sim.notifyNewData();

        assertTrue(board.getButton(13).getAsBoolean()); // axis 0 at -1
        assertFalse(board.getButton(14).getAsBoolean()); // axis 0 at 1
        assertTrue(board.getButton(15).getAsBoolean()); // axis 1 at 1
        assertFalse(board.getButton(16).getAsBoolean()); // axis 1 at -1
    }
}
