package common.hardware.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A flight-style joystick, such as a Logitech Extreme 3D Pro or Thrustmaster T16000M. As with
 * {@link NAR_XboxController}, its axes have a small deadband, forward is positive, and every button
 * is a {@link edu.wpi.first.wpilibj2.command.button.Trigger} to which commands can be bound. The
 * throttle slider reads from 0 at the bottom to 1 at the top, limited to configurable bounds.
 * @author Daniel Wang, Arav Chadha, Mason Lam
 */
public class NAR_Joystick {

    private final Joystick stick;

    private final Trigger[] buttons;

    private final Trigger[] povButtons;

    private double xDeadband = 0.05;
    private double yDeadband = 0.05;
    private double zDeadband = 0.05;
    private double throttleLowerBound = 0.3;
    private double throttleUpperBound = 0.8;

    /**
     * Creates a joystick.
     * @param port The Driver Station port to which the joystick is connected
     */
    public NAR_Joystick(int port) {
        buttons = new Trigger[16];
        povButtons = new Trigger[8];
        stick = new Joystick(port);

        // The Thrustmaster joystick has 16 buttons

        for (int i = 0; i < 16; i++) {
            final int buttonId = i;
            buttons[buttonId] = new Trigger(() -> stick.getRawButton(buttonId + 1)); 
        }
            
        for (int i = 0; i < 8; i++) {
            final int povButtonId = i;
            povButtons[povButtonId] = new Trigger(() -> stick.getPOV() == povButtonId * 45);
        }
            
    }

    /** 
     * Returns the joystick's x-axis value.
     * @return Joystick X on [-1, 1], where -1 is left and 1 is right. The default deadband is 0.05.
    */
    public double getX() {
        return Math.abs(stick.getX()) > xDeadband ? stick.getX() : 0;
    }

    /** 
     * Returns the joystick's y-axis value.
     * @return Joystick Y on [-1, 1], where -1 is backward and 1 is forward. The default deadband is 0.05.
    */
    public double getY() {
        return Math.abs(stick.getY()) > yDeadband ? -stick.getY() : 0;
    }

    /** 
     * Returns the joystick's z-axis value.
     * @return Joystick Z on [-1, 1], where -1 is twist left and 1 is twist right. The default deadband is 0.05.
    */
    public double getZ() {
        return Math.abs(stick.getZ()) > zDeadband ? stick.getZ() : 0;
    }

    /**  
     * Returns the joystick's throttle slider value.
     * @return Throttle on [0, 1], where 0 is the bottom and 1 is the top, limited to the bounds from
     *         {@link #setThrottleBounds(double, double)}. By default these are 0.3 and 0.8, so lower
     *         values return 0.3 and higher values return 0.8.
    */
    public double getThrottle() {
        double mappedThrottle = MathUtil.clamp((1 - stick.getThrottle()) / 2, throttleLowerBound, throttleUpperBound);
        return mappedThrottle;
    }

    /**
     * Sets the throttle bounds for the joystick.
     * @param lowerBound Lowest possible throttle value, on [0, 1].
     * @param upperBound Highest possible throttle value, on [0, 1].
     */
    public void setThrottleBounds(double lowerBound, double upperBound) {
        throttleLowerBound = lowerBound;
        throttleUpperBound = upperBound;
    }

    /**
     * Returns a button on the joystick.
     * @param i The ID of the button
     * @return A trigger to which commands can be bound
     */
    public Trigger getButton(int i) {
        return buttons[i-1];
    }

    /**
     * Returns a POV button on the joystick.
     * @param i The angle of the POV button in increments of 45 degrees
     * @return A trigger to which commands can be bound
     */
    public Trigger getPOVButton(int i) {
        return povButtons[i];
    }

    /**
     * Returns the 0-degree (up) POV button.
     * @return A trigger to which commands can be bound
     */
    public Trigger getUpPOVButton() {
        return getPOVButton(0);
    }

    /**
     * Returns the 90-degree (right) POV button.
     * @return A trigger to which commands can be bound
     */
    public Trigger getRightPOVButton() {
        return getPOVButton(2);
    }

    /**
     * Returns the 180-degree (down) POV button.
     * @return A trigger to which commands can be bound
     */
    public Trigger getDownPOVButton() {
        return getPOVButton(4);
    }

    /**
     * Returns the 270-degree (left) POV button.
     * @return A trigger to which commands can be bound
     */
    public Trigger getLeftPOVButton() {
        return getPOVButton(6);
    }

    /**
     * Sets the x-axis deadband.
     */
    public void setXDeadband(double xDeadband) {
        this.xDeadband = xDeadband;
    }

    /**
     * Sets the y-axis deadband.
     */
    public void setYDeadband(double yDeadband) {
        this.yDeadband = yDeadband;
    }

    /**
     * Sets the z-axis deadband.
     */
    public void setZDeadband(double zDeadband) {
        this.zDeadband = zDeadband;
    }
}