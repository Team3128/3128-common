package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;

public class VoltageController extends ControllerBase implements AutoCloseable {

    public double[] inputRange;
    private double setpoint;

    public VoltageController(double tolerance, double[] inputRange) {
        super(new PIDFFConfig(), tolerance);
        this.inputRange = inputRange;
    }

    @Override
    protected double calculate(double measurement) {
        return setpoint/12;
    }

    @Override
    public void setMeasurementSource(NAR_Motor motor) {
        measurement = () -> motor.getAppliedOutput() * 12;
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        for (NAR_Motor motor : motors) {
            motor.setVolts(setpoint);
        }
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    public boolean atSetpoint() {
        return true;
    }

    @Override
    public void setTolerance(double tolerance) {
    }

    @Override
    public void reset() {
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
