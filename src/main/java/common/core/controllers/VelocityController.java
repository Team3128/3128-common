package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;

public class VelocityController extends ControllerBase implements AutoCloseable {

    private PIDController controller;
    private int plateau = 5;
    private int plateauCount = 0;

    public VelocityController(PIDFFConfig config, double tolerance) {
        super(config, tolerance);

        this.controller = new PIDController(config.getkP(), config.getkI(), config.getkD());
    }

    @Override
    protected double calculate(double measurement) {
        return controller.calculate(measurement);
    }

    @Override
    public void setMeasurementSource(NAR_Motor motor) {
        measurement = () -> motor.getVelocity();
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        controller.setSetpoint(setpoint);
    }

    @Override
    public double getSetpoint() {
        return controller.getSetpoint();
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = controller.atSetpoint();
        if(atSetpoint && plateauCount > plateau){
            plateauCount = 0;
            return true;
        } 
        else if (atSetpoint){
            plateauCount++;
            return false;
        } 
        else{
            plateauCount = 0;
            return false;
        }
    }

    @Override
    public void setTolerance(double tolerance) {
        controller.setTolerance(tolerance);
    }

    @Override
    public void reset() {
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
