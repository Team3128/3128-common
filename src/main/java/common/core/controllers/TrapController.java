package common.core.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TrapController extends ControllerBase {
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State tempSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.Constraints constraints;
    private double minimumInput;
    private double maximumInput;

    /**
     * Create a new object to control PID + FF logic using a trapezoid profile for a subsystem.
     * <p>Sets kP, kI, kD, kS, kV, kG, constraints, period values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kG The gravity gain.
     * @param constraints Constraints for max acceleration and velocity
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public TrapController(double kP, double kI, double kD, double kS, double kV, double kG, TrapezoidProfile.Constraints constraints, double period) {
        super(kP, kI, kD, kS, kV, kG, period);
        this.constraints = constraints;
    }

    /**
     * Create a new object to control PID + FF logic using a trapezoid profile for a subsystem.
     * <p>Sets kP, kI, kD, kS, kV, kG, constraints values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kG The gravity gain.
     * @param constraints Constraints for max acceleration and velocity
     */
    public TrapController(double kP, double kI, double kD, double kS, double kV, double kG, TrapezoidProfile.Constraints constraints) {
        this(kP, kI, kD, kS, kV, kG, constraints, 0.02);
    }

    /**
     * Create a new object to control PID logic using a trapezoid profile for a subsystem.
     * <p>Sets kP, kI, kD, constraints, period values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param constraints Constraints for max acceleration and velocity
     * @param period The controller's update rate in seconds. Must be non-zero and positive
     */
    public TrapController(double kP, double kI, double kD, TrapezoidProfile.Constraints constraints, double period) {
        this(kP, kI, kD, 0, 0, 0, constraints, period);
    }

    /**
     * Create a new object to control PID logic using a trapezoid profile for a subsystem.
     * <p>Sets kP, kI, kD, constraints, values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param constraints Constraints for max acceleration and velocity
     */
    public TrapController(double kP, double kI, double kD, TrapezoidProfile.Constraints constraints) {
        this(kP, kI, kD, constraints, 0.02);
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
        final var profile = new TrapezoidProfile(constraints, setpoint, tempSetpoint);
        tempSetpoint = profile.calculate(getPeriod());
        final double PID_OUTPUT = controller.calculate(measurement, tempSetpoint.position);
        final double ff_output = Math.copySign(getkS(), PID_OUTPUT) + getkV() * tempSetpoint.velocity + getkG() * kG_Function.getAsDouble();
        return PID_OUTPUT + ff_output;
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
        super.reset();
        setpoint = new TrapezoidProfile.State(measurement.getAsDouble(), 0);
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
