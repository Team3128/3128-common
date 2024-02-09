package common.core.controllers;

import java.util.function.DoubleSupplier;

import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Team 3128's streamlined {@link ProfiledPIDController} class.
 * 
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public class TrapController extends ControllerBase {
    public static boolean shouldLog = false;

    private DoubleSupplier systemVelocity;

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State tempSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State prevSetpoint = new TrapezoidProfile.State();
    private SimpleMotorFeedforward feedforward;
    private TrapezoidProfile profile; 
    private double minimumInput;
    private double maximumInput;
    /**
     * Create a new object to control PID + FF logic using a trapezoid profile for a subsystem.
     * <p>Sets kP, kI, kD, kS, kV, kA, kG, constraints, period values.
     * 
     * @param config PIDFFConfig object containing PID and Feedforward constants.
     * @param constraints Constraints for max acceleration and velocity
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public TrapController(PIDFFConfig config, TrapezoidProfile.Constraints constraints, double period) {
        super(config, period);
        profile = new TrapezoidProfile(constraints);
        feedforward = new SimpleMotorFeedforward(getkS(), getkV(), getkA());
        systemVelocity = ()-> 0;
    }

    /**
     * Create a new object to control PID + FF logic using a trapezoid profile for a subsystem.
     * <p>Sets kP, kI, kD, kS, kV, kA, kG, constraints values.
     * 
     * @param config PIDFFConfig object containing PID and Feedforward constants.
     * @param constraints Constraints for max acceleration and velocity
     */
    public TrapController(PIDFFConfig config, TrapezoidProfile.Constraints constraints) {
        this(config, constraints, 0.02);
    }

    /**
     * Sets the velocity source for resetting the controller.
     * @param velocitySrc Returns the current velocity of the system.
     */
    public void setVelocitySource(DoubleSupplier velocitySrc) {
        this.systemVelocity = velocitySrc;
    }

    /**
     * Sets a new setpoint, and returns the next output of the controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint The new setpoint.
     * @return The next controller output.
     */
    public double calculate(TrapezoidProfile.State setpoint, double measurement) {
        setSetpoint(setpoint);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    @Override
    public double calculate(double measurement) {
        if (controller.isContinuousInputEnabled()) {
            // Get error which is the smallest distance between goal and measurement
            double errorBound = (maximumInput - minimumInput) / 2.0;
            double goalMinDistance =
                MathUtil.inputModulus(setpoint.position - measurement, -errorBound, errorBound);
            double setpointMinDistance =
                MathUtil.inputModulus(tempSetpoint.position - measurement, -errorBound, errorBound);
      
            // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
            // may be outside the input range after this operation, but that's OK because the controller
            // will still go there and report an error of zero. In other words, the setpoint only needs to
            // be offset from the measurement by the input range modulus; they don't need to be equal.
            setpoint.position = goalMinDistance + measurement;
            tempSetpoint.position = setpointMinDistance + measurement;
        }
        if (shouldLog) NarwhalDashboard.getInstance().sendMessage("Position: " + tempSetpoint.position + "Velocity: " + tempSetpoint.velocity);
        tempSetpoint = profile.calculate(getPeriod(), tempSetpoint, setpoint);
        controller.setSetpoint(tempSetpoint.position);
        final double output = super.calculate(measurement);
        prevSetpoint = tempSetpoint;
        return output;
    }

    /**
     * Returns the PID output of the controller.
     * @param measurement the current measurement of the process variable.
     * @return The controller output due to PID terms.
     */
    @Override
    public double calculatePID(double measurement) {
        return controller.calculate(measurement, tempSetpoint.position);
    }

    /**
     * Returns the Feed Forward output of the controller.
     * <p>Uses kS, kV, kA, and kG
     * @param pidOutput the current measurement of the process variable.
     * @return The controller output due to Feed Forward terms.
     */
    @Override
    public double calculateFF(double pidOutput) {
        final double simpleGain = feedforward.calculate(prevSetpoint.velocity, tempSetpoint.velocity, getPeriod());
        final double gravityGain = getkG() * kG_Function.getAsDouble();
        if (shouldLog) NarwhalDashboard.getInstance().sendMessage("Simple Gain: " + simpleGain + "Gravity Gain: " + gravityGain);
        return simpleGain + gravityGain;
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
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return True if the error is within the tolerance of the error.
     */
    @Override
    public boolean atSetpoint() {
        return super.atSetpoint() && setpoint.equals(tempSetpoint);
    }

    /**
     * Sets the setpoint for the controller.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(TrapezoidProfile.State setpoint) {
        this.setpoint = setpoint;
        feedforward = new SimpleMotorFeedforward(getkS(), getkV(), getkA());
        reset();
    }

    /**
     * Sets the setpoint for the controller.
     *
     * @param setpoint The desired setpoint.
     */
    @Override
    public void setSetpoint(double setpoint) {
        setSetpoint(new TrapezoidProfile.State(setpoint, 0));
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    @Override
    public double getSetpoint() {
        return setpoint.position;
    }

    /** Resets the previous error and the integral term. */
    @Override
    public void reset() {
        tempSetpoint = new TrapezoidProfile.State(measurement.getAsDouble(), systemVelocity.getAsDouble());
        super.reset();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ProfiledPIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("goal", () -> getSetpoint(), this::setSetpoint);
    }
}
