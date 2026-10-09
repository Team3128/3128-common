package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public abstract class ControllerBase implements Sendable {

    protected final List<NAR_Motor> motors = new ArrayList<NAR_Motor>();
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

    protected abstract double calculate(double measurement);

    protected double calculateFF(double pidOutput){
        final double staticGain = !atSetpoint() ? Math.copySign(config.getkS(), pidOutput) : 0;
        final double velocityGain = config.getkV() * getSetpoint();
        final double gravityGain = config.getkG() * config.getkG_Function().getAsDouble();
        return staticGain + velocityGain + gravityGain;
    }

    public void useOutput() {
        if (atSetpoint()) {
            disable();
        }
        else if (isEnabled()) {
            final double output = calculate(getMeasurement()) + calculateFF(calculate(getMeasurement()));
            for (NAR_Motor motor : motors) {
                motor.set(output);
            }
        }
    }

    public double getMeasurement() {
        return measurement.getAsDouble();
    }

    public abstract void setTolerance(double tolerance);

    public abstract boolean atSetpoint();

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
    public abstract void reset();

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
