package common.hardware.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Custom GenericHID class for team 3128's button board setup
 * @since 2023 Charged Up
 * @author Peter Ma, Arav Chadha, Mason Lam
 */
public class NAR_ButtonBoard {

    private GenericHID device;

    private Trigger[] buttons;

    /**
     * Creates a new button board object.
     * @param port The port on DriverStation the button board is connected to.
     */
    public NAR_ButtonBoard(int port) {
        buttons = new Trigger[16];
        device = new GenericHID(port);

        // 2023 buttonBoard has 12 buttons & 4 axis direction
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
     * Returns a button on the board
     * @param i The ID of the button
     * @return Trigger to store the command to run
     */
    public Trigger getButton(int i) {
        return buttons[i-1];
    }
}