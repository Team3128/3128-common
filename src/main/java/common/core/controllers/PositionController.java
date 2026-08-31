package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;

public class PositionController extends ControllerBase implements AutoCloseable {

    private PIDController controller;
    public double[] inputRange;

    public PositionController(PIDFFConfig config, double tolerance, double[] inputRange) {
        super(config, tolerance);
        this.inputRange = inputRange;

        this.controller = new PIDController(config.getkP(), config.getkI(), config.getkD());
    }

    @Override
    protected double calculate(double measurement) {
        return controller.calculate(measurement) + config.getFF();
    }

    @Override
    public void setMeasurementSource(NAR_Motor motor) {
        measurement = () -> motor.getPosition();
    }

    @Override
    public void setSetpoint(double setpoint) {
        controller.setSetpoint(Math.max(inputRange[0], Math.min(setpoint, inputRange[1])));
    }

    @Override
    public void setTolerance(double tolerance) {
        super.setTolerance(tolerance);
        controller.setTolerance(tolerance);
    }

    @Override
    public void reset() {
        super.reset();
        controller.reset();
    }

    @Override
    public void enable() {
        super.enable();
        controller.reset();
    }

    @Override
    public void close() {
        controller.close();
    }

    public void enableContinuousInput() {
        controller.enableContinuousInput(inputRange[0], inputRange[1]);
    }

    public double[] getInputRange() {
        return inputRange;
    }
}
