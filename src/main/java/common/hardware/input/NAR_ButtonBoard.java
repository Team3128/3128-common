package common.hardware.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Team 3128's operator button board: 12 buttons and a joystick, read as 16 buttons to which commands
 * can be bound. Buttons 1 to 12 are the physical buttons; 13 to 16 are the joystick fully deflected
 * left, right, down and up.
 *
 * <pre>{@code
 * buttonBoard.getButton(3).onTrue(superstructure.setStateCommand(SHOOT));
 * }</pre>
 * @since 2023 Charged Up
 * @author Peter Ma, Arav Chadha, Mason Lam
 */
public class NAR_ButtonBoard {

    private GenericHID device;

    private Trigger[] buttons;

    /**
     * Creates a button board.
     * @param port The Driver Station port to which the button board is connected.
     */
    public NAR_ButtonBoard(int port) {
        buttons = new Trigger[16];
        device = new GenericHID(port);

        // The 2023 button board has 12 buttons and 4 axis directions
        for (int i = 0; i < 12; i++) {
            int buttonId = i;
            buttons[buttonId] = new Trigger(() -> device.getRawButton(buttonId + 1)); 
        }

        buttons[12] = new Trigger(() -> device.getRawAxis(0) == -1.0);
        buttons[13] = new Trigger(() -> device.getRawAxis(0) == 1.0);
        buttons[14] = new Trigger(() -> device.getRawAxis(1) == 1.0);
        buttons[15] = new Trigger(() -> device.getRawAxis(1) == -1.0);
            
    }

    /**
     * Returns a button on the board, to which commands can be bound.
     * @param i From 1 to 16; see the class comment for the mapping
     */
    public Trigger getButton(int i) {
        return buttons[i-1];
    }
}