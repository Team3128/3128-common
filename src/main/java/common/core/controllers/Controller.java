package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;
/**
 * Team 3128's streamlined {@link PIDController} class.
 * 
 * @since 2023 Charged Up
 * @author Mason Lam, William Yuan
 */
public class Controller extends ControllerBase {

    /**
     * Setpoint types
     */
    public enum Type {
        VELOCITY,
        POSITION
    }

    private final Type type;

    /**
     * Create a new object to control PID + FF logic for a subsystem.
     * <p>Sets kP, kI, kD, kS, kV, kG, period values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kG The gravity gain.
     * @param type The type of setpoint used by the controller
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public Controller(double kP, double kI, double kD, double kS, double kV, double kG, Type type, double period) {
        super(kP, kI, kD, kS, kV, 0, kG, period);
        this.type = type;
    }

    /**
     * Create a new object to control PID + FF logic for a subsystem.
     * <p>Sets kP, kI, kD, kS, kV, kG values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kG The gravity gain.
     * @param type The type of setpoint used by the controller
     */
    public Controller(double kP, double kI, double kD, double kS, double kV, double kG, Type type) {
        this(kP, kI, kD, kS, kV, kG, type, 0.02);
    }

    /**
     * Create a new object to control PID logic for a subsystem.
     * <p>Sets kP, kI, kD, period values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public Controller(double kP, double kI, double kD, double period) {
        this(kP, kI, kD, 0, 0, 0, Type.POSITION, period);
    }

    /**
     * Create a new object to control PID logic for a subsystem.
     * <p>Sets kP, kI, kD values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     */
    public Controller(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.02);
    }

    /**
     * Sets a motor's velocity/position as the measurement source
     * @param motor {@link NAR_Motor} motor type.
     */
    public void setMeasurementSource(NAR_Motor motor) {
        if (type == Type.VELOCITY) {
            setMeasurementSource(()-> motor.getVelocity());
            return;
        }
        setMeasurementSource(()-> motor.getPosition());
    }

    /**
     * Returns the PID output of the controller.
     * @param measurement the current measurement of the process variable.
     * @return The controller output due to PID terms.
     */
    @Override
    public double calculatePID(double measurement) {
        return controller.calculate(measurement);
    }

    /**
     * Returns the Feed Forward output of the controller.
     * <p>Uses kS, kG, and optionally kV
     * @param pidOutput the current measurement of the process variable.
     * @return The controller output due to Feed Forward terms.
     */
    @Override
    public double calculateFF(double pidOutput) {
        final double staticGain = Math.copySign(getkS(), pidOutput);
        final double velocityGain = (type == Type.VELOCITY) ? getkV() * getSetpoint() : 0;
        final double gravityGain = getkG() * kG_Function.getAsDouble();
        return staticGain + velocityGain + gravityGain;
    }

    /**
     * Returns the type of setpoint used by the controller
     * @return {@link Type}: VELOCITY, POSITION, NONE
    */
    public Type getType() {
        return type;
    }
}
