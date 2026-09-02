package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;

public class VoltageController extends ControllerBase implements AutoCloseable {

    public double[] inputRange;
    private double setpoint;

    public VoltageController(double tolerance, double[] inputRange) {
        super(new PIDFFConfig(), tolerance);
        this.inputRange = inputRange;
    }

    @Override
    public void useOutput() {
        if (isEnabled() && atSetpoint()) disable();
        for (NAR_Motor motor : motors) {
            motor.setVolts(setpoint);
        }
    }

    @Override
    public void setMeasurementSource(NAR_Motor motor) {
        measurement = () -> motor.getAppliedOutput() * 12;
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void setTolerance(double tolerance) {
        super.setTolerance(tolerance);
    }

    @Override
    public void reset() {
        super.reset();
    }

    @Override
    public void enable() {
        super.enable();
    }

    @Override
    public void close() {}

    public double[] getInputRange() {
        return inputRange;
    }
}
