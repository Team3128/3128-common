package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public abstract class ControllerBase implements Sendable {

    private final List<NAR_Motor> motors = new ArrayList<NAR_Motor>();
    protected final PIDFFConfig config;
    protected double tolerance;

    public double setpoint;
    public boolean enabled = false;

    protected DoubleSupplier measurement;

    public ControllerBase(PIDFFConfig config, double tolerance) {
        this.config = config;
        this.tolerance = tolerance;

        requireNonNullParam(config, "config", "Controller");
    }

    public void addMotor(NAR_Motor motor) {
        motors.add(motor);
    }

    public void setMeasurementSource(NAR_Motor motor) {}

    protected double calculate(double measurement) { return 0; }

    public void useOutput() {
        if (isEnabled() && atSetpoint()) disable();
        final double output = MathUtil.clamp(calculate(getMeasurement()), -12, 12);
        for (NAR_Motor motor : motors) {
            motor.setVolts(output);
        }
    }

    public double getMeasurement() {
        return measurement.getAsDouble();
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public boolean atSetpoint() {
        return Math.abs(getMeasurement() - getSetpoint()) < tolerance;
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return setpoint;
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        for (NAR_Motor motor : motors) {
            motor.setVolts(0);
        }
    }

    public PIDFFConfig getConfig() {
        return this.config;
    }

    public void enable() {
        enabled = true;
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
        builder.addDoubleProperty("p", config::getkP, config::setkP);
        builder.addDoubleProperty("i", config::getkI, config::setkI);
        builder.addDoubleProperty("d", config::getkD, config::setkD);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
