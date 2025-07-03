package common.hardware.input;

import java.util.HashMap;

import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper for the WPILib XboxController class.
 * @since 2022 Rapid React
 * @author Arav Chadha, Mason Lam
 */
public class NAR_XboxController extends XboxController {

    /**
     * Enum to represent buttons
     */
    public enum XboxButton {
        kLeftBumper(5, false),
        kRightBumper(6, false),
        kLeftStick(9, false),
        kRightStick(10, false),
        kA(1, false),
        kB(2, false),
        kX(3, false),
        kY(4, false),
        kBack(7, false),
        kStart(8, false),
        kLeftTrigger(2, true),
        kRightTrigger(3, true);

        private final int value;
        private final boolean isAxis;

        private XboxButton(int value, boolean isAxis) {
            this.value = value;
            this.isAxis = isAxis;
        }
    }

    private double leftXDeadband = 0.05;
    private double leftYDeadband = 0.05;
    private double rightXDeadband = 0.05;
    private double rightYDeadband = 0.05;
    
    private HashMap<XboxButton, Trigger> buttons;
    private Trigger[] povButtons;

    /**
     * Creates an XboxController object
     * @param port Port on driver station for controller
     */
    public NAR_XboxController(int port) {
        super(port);
        buttons = new HashMap<XboxButton, Trigger>();
        for (final XboxButton button : XboxButton.values()) {
            if (button.isAxis) {
                buttons.put(button, new Trigger(()-> getRawAxis(button.value) >= 0.5));
                continue;
            }
            buttons.put(button, new Trigger(()-> getRawButton(button.value)));
        }

        povButtons = new Trigger[8];
        for (int i = 0; i < 8; i++) {
            final int n = i;
            povButtons[i] = new Trigger (() -> getPOV() == n * 45);
        }
    }

    /**
     * Returns the XboxController Button
     * @param button Type of button.
     * @return Trigger object to add commands to.
     */
    public Trigger getButton(XboxButton button) {
        return buttons.get(button);
    }

    /**
     * Returns a POV button on the controller
     * @param index The angle of the POV button in increments of 45 degrees, positive direction is clockwise
     * @return Trigger to store the command to run
     */
    public Trigger getPOVButton(int index) {
        return povButtons[index];
    }

    /**
     * Returns the 0 degree or top POV button.
     * @return Trigger object to add commands to.
     */
    public Trigger getUpPOVButton() {
        return getPOVButton(0);
    }

    /**
     * Returns the 90 degree or right POV button.
     * @return Trigger object to add commands to.
     */
    public Trigger getRightPOVButton() {
        return getPOVButton(2);
    }

    /**
     * Returns the 180 degree or bottom POV button.
     * @return Trigger object to add commands to.
     */
    public Trigger getDownPOVButton() {
        return getPOVButton(4);
    }

    /**
     * Returns the 270 degree or left POV button.
     * @return Trigger object to add commands to.
     */
    public Trigger getLeftPOVButton() {
        return getPOVButton(6);
    }

    /**
     * Returns the x-axis value of the right joystick.
     * @return Right joystick X on [-1, 1], -1 is left, 1 is right.
     */
    @Override
    public double getRightX() {
        return Math.abs(super.getRightX()) > rightXDeadband ? super.getRightX() : 0;
    }

    /**
     * Returns the y-axis value of the right joystick.
     * @return Right joystick Y on [-1, 1], -1 is down, 1 is up.
     */
    @Override
    public double getRightY() {
        return Math.abs(super.getRightY()) > rightYDeadband ? -super.getRightY() : 0;
    }

    /**
     * Returns the x-axis value of the left joystick.
     * @return Left joystick X on [-1, 1], -1 is left, 1 is right.
     */
    @Override
    public double getLeftX() {
        return Math.abs(super.getLeftX()) > leftXDeadband ? super.getLeftX() : 0;
    }

    /**
     * Returns the y-axis value of the left joystick.
     * @return Left joystick Y on [-1, 1], -1 is down, 1 is up.
     */
    @Override
    public double getLeftY() {
        return Math.abs(super.getLeftY()) > leftYDeadband ? -super.getLeftY() : 0;
    }

    /**
     * Sets the horizontal deadband value for the right joystick.
     * <p>Deadband reduces noise by requiring input above a certain threshold.
     * @param deadband Value for deadband to be set to
     */
    public void setRightXDeadband(double deadband) {
        rightXDeadband = deadband;
    }

    /**
     * Sets the vertical deadband value for the right joystick.
     * <p>Deadband reduces noise by requiring input above a certain threshold.
     * @param deadband Value for deadband to be set to
     */
    public void setRightYDeadband(double deadband) {
        rightYDeadband = deadband;
    }
    
    /**
     * Sets the horizontal deadband value for the left joystick.
     * <p>Deadband reduces noise by requiring input above a certain threshold.
     * @param deadband Value for deadband to be set to
     */
    public void setLeftXDeadband(double deadband) {
        leftXDeadband = deadband;
    }

    /**
     * Sets the vertical deadband value for the left joystick.
     * <p>Deadband reduces noise by requiring input above a certain threshold.
     * @param deadband Value for deadband to be set to
     */
    public void setLeftYDeadband(double deadband) {
        leftYDeadband = deadband;
    }

    /**
     * Vibrates the controller
     */
    public void startVibrate() {
        setRumble(RumbleType.kBothRumble, 0.8);
    }

    /**
     * Stops vibrating the controller
     */
    public void stopVibrate() {
        setRumble(RumbleType.kBothRumble, 0);
    }

    public void initShuffleboard() {
        for(XboxButton button : buttons.keySet()) {
            NAR_Shuffleboard.addData("Controller", button.name(), ()-> buttons.get(button).getAsBoolean(), (button.ordinal() % 6), button.ordinal() / 6);
        }
    }
}