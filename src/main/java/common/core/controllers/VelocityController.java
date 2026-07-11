package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;

public class VelocityController extends ControllerBase implements AutoCloseable {

    private PIDController controller;

    public VelocityController(PIDFFConfig config, double tolerance) {
        super(config, tolerance);

        this.controller = new PIDController(config.getkP(), config.getkI(), config.getkD());
    }

    @Override
    protected double calculate(double measurement) {
        return controller.calculate(measurement) + config.getFF();
    }

    @Override
    public void setMeasurementSource(NAR_Motor motor) {
        measurement = () -> motor.getVelocity();
    }

    @Override
    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
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
}
