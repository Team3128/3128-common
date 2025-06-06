package common.core.controllers;

import java.util.LinkedList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import common.hardware.motorcontroller.NAR_Motor;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/**
 * Team 3128's abstract controller class.
 * 
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public abstract class ControllerBase implements Sendable, AutoCloseable {

    protected final PIDController controller;
    private final LinkedList<DoubleConsumer> consumers = new LinkedList<DoubleConsumer>();
    protected DoubleSupplier measurement;

    protected PIDFFConfig config;

    protected double inputMin, inputMax, outputMin, outputMax;

    protected boolean enabled;
    protected boolean disabledAtSetpoint;

    /**
     * Creates a base controller object to control motion.
     * <p>Sets kP, kI, kD, kS, kV, kA, kG, constraints, period values.
     * @param config PIDFFConfig object containing PID and Feedforward constants.
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public ControllerBase(PIDFFConfig config, double period) {
        controller = new PIDController(config.kP, config.kI, config.kD, period);
        this.config = config;

        this.inputMin = Double.NEGATIVE_INFINITY;
        this.inputMax = Double.POSITIVE_INFINITY;
        this.outputMin = Double.NEGATIVE_INFINITY;
        this.outputMax = Double.POSITIVE_INFINITY;

        this.enabled = false;
    }

    public ControllerBase(PIDFFConfig config) {
        this(config, 0.02);
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
        addOutput(motor::setVolts);
        setOutputRange(-12, 12);
    }

    /**
     * Adds a motor for controller to send output
     * @param motor {@link MotorController} motor type.
     */
    public void addMotor(MotorController motor) {
        addOutput(motor::setVoltage);
        setOutputRange(-12, 12);
    }

    /**
     * Adds an output source for controller to send output
     * @param output Method that takes the controller's output
     */
    public void addOutput(DoubleConsumer output) {
        consumers.add(output);
    }

    public void configureFeedback(DoubleSupplier measurement, DoubleConsumer output){
        setMeasurementSource(measurement);
        addOutput(output);
    }

    public abstract void configureFeedback(NAR_Motor motor);

    /**
     * Returns the measurement of the controller.
     * @return The measurement of the controller.
     */
    public double getMeasurement() {
        return measurement.getAsDouble();
    }

    /**
     * Calculates and sends output voltage.
     * @return Calculated output voltage.
     */
    public double useOutput() {
        if(isEnabled() && atSetpoint() && disabledAtSetpoint) disable();
        if (measurement == null || !isEnabled()) return 0;
        final double output = MathUtil.clamp(calculate(getMeasurement()), outputMin, outputMax);
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
        controller.setPID(kp, ki, kd);
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
     * Sets the minimum and maximum values expected from the input.
     *
     * <p>Input values outside of this range will be clamped.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void setInputRange(double minimumInput, double maximumInput) {
        inputMin = minimumInput;
        inputMax = maximumInput;
    }

    /**
     * Returns the minimum and maximum values expected from the input.
     * 
     * <p>First element is inputMin.
     * <p>Second element is inputMax.
     * 
     * @return double[] with inputMin and inputMax.
     */
    public double[] getInputRange() {
        return new double[] {inputMin, inputMax};
    }

    /**
     * Sets the minimum and maximum values of the controller output.
     *
     * <p>Output values outside of this range will be clamped.
     *
     * @param minimumOutput The minimum value to write.
     * @param maximumOutput The maximum value to write.
     */
    public void setOutputRange(double minimumOutput, double maximumOutput) {
        outputMin = minimumOutput;
        outputMax = maximumOutput;
    }

    public void setConstraints(Constraints constraints) {
        setOutputRange(-constraints.maxVelocity, constraints.maxVelocity);
    }

    /**
     * Returns the minimum and maximum values expected from the output.
     * 
     * <p>First element is outputMin.
     * <p>Second element is outputMax.
     * 
     * @return double[] with outputMin and outputMax.
     */
    public double[] getOutputRange() {
        return new double[] {outputMin, outputMax};
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
     * @param measurementTolerance Position error which is tolerable.
     */
    public void setTolerance(double measurementTolerance) {
        setTolerance(measurementTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param measurementTolerance Position error which is tolerable.
     * @param measurementROCTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double measurementTolerance, double measurementROCTolerance) {
        controller.setTolerance(measurementTolerance, measurementROCTolerance);
    }

    public void setDisableAtSetpoint(boolean disabledAtSetpoint) {
        this.disabledAtSetpoint = disabledAtSetpoint;
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
        controller.setSetpoint(MathUtil.clamp(setpoint, inputMin, inputMax));
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
        for (final DoubleConsumer consumer : consumers) {
            consumer.accept(0);
        }
    }

    public double getError() {
        return getMeasurement() - getSetpoint();
    }

    public PIDFFConfig getConfig() {
        return this.config;
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

    public void enable() {
        enabled = true;
        controller.reset();
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }

    /**
     * Closes the Controller.
     */
    @Override
    public void close() {
        controller.close();
    }
}
