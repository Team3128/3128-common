package common.core.controllers;

import java.util.LinkedList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Team 3128's abstract controller class.
 * 
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public abstract class ControllerBase implements Sendable {

    protected final PIDController controller;
    private final LinkedList<DoubleConsumer> consumers = new LinkedList<DoubleConsumer>();
    protected DoubleSupplier measurement;

    protected DoubleSupplier kS, kV, kA, kG;
    protected DoubleSupplier kG_Function = ()-> 1;

    /**
     * Creates a base controller object to control motion.
     * <p>Sets kP, kI, kD, kS, kV, kA, kG, constraints, period values.
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     * @param kG The gravity gain.
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public ControllerBase(double kP, double kI, double kD, double kS, double kV, double kA, double kG, double period) {
        controller = new PIDController(kP, kI, kD, period);
        this.kS = ()-> kS;
        this.kV = ()-> kV;
        this.kA = ()-> kA;
        this.kG = ()-> kG;

        kG_Function = ()-> 1;
    }

    /**
     * Sets the measurement source for PID control.
     * @param measurement Returns the current state of the system.
     */
    public void setMeasurementSource(DoubleSupplier measurement) {
        this.measurement = measurement;
    }

    /**
     * Adds a motor for controller to send output
     * @param motor {@link NAR_Motor} motor type.
     */
    public void addMotor(NAR_Motor motor) {
        addOutput(motor::set);
    }

    /**
     * Adds a motor for controller to send output
     * @param motor {@link MotorController} motor type.
     */
    public void addMotor(MotorController motor) {
        addOutput(motor::set);
    }

    /**
     * Adds an output source for controller to send output
     * @param output Method that takes the controller's output
     */
    public void addOutput(DoubleConsumer output) {
        consumers.add(output);
    }

    /**
     * Calculates and sends output.
     * @return Calculated output.
     */
    public double useOutput() {
        if (measurement == null) return 0;
        final double output = calculate(measurement.getAsDouble());
        for (final DoubleConsumer consumer : consumers) {
            consumer.accept(output);
        }
        return output;
    }


    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint The new setpoint of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement) {
        final double pidOutput = calculatePID(measurement);
        return pidOutput + calculateFF(pidOutput);
    }

    /**
     * Returns the PID output of the controller.
     * @param measurement the current measurement of the process variable.
     * @return The controller output due to PID terms.
     */
    protected abstract double calculatePID(double measurement);

    /**
     * Returns the Feed Forward output of the controller.
     * @param pidOutput the current measurement of the process variable.
     * @return The controller output due to Feed Forward terms.
     */
    protected abstract double calculateFF(double pidOutput);

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public void setPID(double kp, double ki, double kd) {
        setP(kp);
        setI(ki);
        setD(kd);
    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kp proportional coefficient
     */
    public void setP(double kp) {
        controller.setP(kp);
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki integral coefficient
     */
    public void setI(double ki) {
        controller.setI(ki);
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd differential coefficient
     */
    public void setD(double kd) {
        controller.setD(kd);
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return controller.getP();
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return controller.getI();
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return controller.getD();
    }

    /**
     * Enables continuous input.
     *
     * <p>Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        controller.enableContinuousInput(minimumInput, maximumInput);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        controller.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Returns true if the error is within the tolerance of the setpoint.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint) {
        reset();
        controller.setSetpoint(setpoint);
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return controller.getSetpoint();
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        controller.reset();
    }

    /**
     * Sets the static gain.
     * @param kS The constant power required to overcome static friction as a double.
     */
    public void setkS(double kS) {
        setkS(()-> kS);
    }

    /**
     * Sets the static gain.
     * @param kS Modifies kS based on the function.
     */
    public void setkS(DoubleSupplier kS) {
        this.kS = kS;
    }

    /**
     * Sets the velocity gain.
     * @param kV The constant power required to maintain a set velocity as a double.
     */
    public void setkV(double kV) {
        setkV(()-> kV);
    }

    /**
     * Sets the velocity gain.
     * @param kV Modifies kV based on the function.
     */
    public void setkV(DoubleSupplier kV) {
        this.kV = kV;
    }

    /**
     * Sets the velocity gain.
     * @param kA Modifies kV based on the function.
     */
    public void setkA(DoubleSupplier kA) {
        this.kA = kA;
    }

    /**
     * Sets the gravity gain.
     * @param kG The constant power required to overcome gravity as a double.
     */
    public void setkG(double kG) {
        this.kG = ()-> kG;
    }

    /**
     * Sets the gravity gain.
     * @param kG Modifies kG based on the function.
     */
    public void setkG(DoubleSupplier kG) {
        this.kG = kG;
    }

    /**
     * Sets a function to modify gravity gain based on another factor.
     * <p>Example use would be a Pivot which would have gravity gain dependent on angle.
     * @param kG_Function DoubleSupplier with specified logic.
     */
    public void setkG_Function(DoubleSupplier kG_Function) {
        this.kG_Function = kG_Function;
    }

    /**
     * Returns static gain.
     * @return returns kS as a double.
     */
    public double getkS() {
        return kS.getAsDouble();
    }

    /**
     * Returns velocity gain.
     * @return returns kV as a double.
     */
    public double getkV() {
        return kV.getAsDouble();
    }

    /**
     * Returns acceleration gain.
     * @return returns kA as a double.
     */
    public double getkA() {
        return kA.getAsDouble();
    }
    
    /**
     * Returns gravity gain.
     * @return returns kG as a double.
     */
    public double getkG() {
        return kG.getAsDouble();
    }

    /**
     * Gets the period of this controller.
     *
     * @return The period of the controller.
     */
    public double getPeriod() {
        return controller.getPeriod();
    }

    /**
     * Returns the PIDController responsible for PID output
     *
     * @return A PIDController object
     */
    public PIDController getController() {
        return controller;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
