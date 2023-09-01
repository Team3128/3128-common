package frc.team3128.common.hardware.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper for the WPILib {@link Joystick} class. Works with Logitech Extreme 3D Pro and Thrustmaster T16000M.
 * @author Daniel Wang, Arav Chadha, Mason Lam
 */
public class NAR_Joystick {

    private final Joystick stick;

    private final Trigger[] buttons;

    /**
     * POV convention: 0 = up, 45 = top right, 90 = right, 135 = buttom right, 180 = down, 225 = bottom left, 270 = left, 315 = top left
     * We assign indices as angle / 45 [0,7]
     */
    private final Trigger[] povButtons;

    private double xDeadband = 0.05;
    private double yDeadband = 0.05;
    private double zDeadband = 0.05;
    private double throttleLowerBound = 0.3;
    private double throttleUpperBound = 0.8;

    /**
     * Creates a new NAR_Joystick object
     * @param deviceNumber The port the joystick is connected on driver station
     */
    public NAR_Joystick(int deviceNumber) {
        buttons = new Trigger[16];
        povButtons = new Trigger[8];
        stick = new Joystick(deviceNumber);

        // Thrustmaster joystick has 16 buttons

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
     * Returns the Joystick's x axis value
     * @return Joystick X on [-1, 1], -1 is left, 1 is right - default deadband is 0.05 
    */
    public double getX() {
        return Math.abs(stick.getX()) > xDeadband ? stick.getX() : 0;
    }

    /** 
     * Returns the Joystick's y axis value
     * @return Joystick Y on [-1, 1], -1 is backward, 1 is forward - default deadband is 0.05 
    */
    public double getY() {
        return Math.abs(stick.getY()) > yDeadband ? -stick.getY() : 0;
    }

    /** 
     * Returns the Joystick's z axis value
     * @return Joystick Z on [-1, 1], -1 is twist left, 1 is twist right - default deadband is 0.05 
    */
    public double getZ() {
        return Math.abs(stick.getZ()) > zDeadband ? stick.getZ() : 0;
    }

    /**  
     * Returns the Joystick's throttle slider value
     * @return Throttle on [0, 1] where 0 is throttle at bottom, 1 is throttle at top - Default lower bound is 0.3, upper bound is 0.8, so anything below 0.3 returns 0.3, anything above 0.8 returns 1. 
    */
    public double getThrottle() {
        double mappedThrottle = MathUtil.clamp((1 - stick.getThrottle()) / 2, throttleLowerBound, throttleUpperBound);
        return mappedThrottle;
    }

    /**
     * Returns the button of the Joystick
     * @param i The ID of the button
     * @return Trigger to store the command to run
     */
    public Trigger getButton(int i) {
        return buttons[i-1];
    }

    /**
     * Returns the POV button of the joystick
     * @param i The angle of the POV button in increments of 45 degrees
     * @return Trigger to store the command to run
     */
    public Trigger getPOVButton(int i) {
        return povButtons[i];
    }

    /**
     * Returns the 0 degree or up POV button
     * @return Trigger to store the command to run
     */
    public Trigger getUpPOVButton() {
        return getPOVButton(0);
    }

    /**
     * Returns the 90 degree or right POV button
     * @return Trigger to store the command to run
     */
    public Trigger getRightPOVButton() {
        return getPOVButton(2);
    }

    /**
     * Returns the 180 degree or down POV button
     * @return Trigger to store the command to run
     */
    public Trigger getDownPOVButton() {
        return getPOVButton(4);
    }

    /**
     * Returns the 270 degree or left POV button
     * @return Trigger to store the command to run
     */
    public Trigger getLeftPOVButton() {
        return getPOVButton(6);
    }

    /**
     * Set the x-axis deadband
     */
    public void setXDeadband(double xDeadband) {
        this.xDeadband = xDeadband;
    }

    /**
     * Set the y-axis deadband
     */
    public void setYDeadband(double yDeadband) {
        this.yDeadband = yDeadband;
    }

    /**
     * Set the z-axis deadband
     */
    public void setZDeadband(double zDeadband) {
        this.zDeadband = zDeadband;
    }
}