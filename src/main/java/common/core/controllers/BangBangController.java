package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.util.sendable.SendableBuilder;

public class BangBangController extends ControllerBase {

    private edu.wpi.first.math.controller.BangBangController controller;

    public BangBangController(PIDFFConfig config, double tolerance) {
        super(config, tolerance);
    }

    @Override
    protected double calculate(double measurement) {
        return controller.calculate(measurement) + calculateFF(controller.calculate(measurement));
    }

    @Override
    public double calculateFF(double pidOutput){
        if (atSetpoint()){
            return config.getkS() + config.getkV() *getMeasurement();
        } else {
            return 0;
        }
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
        controller.setTolerance(tolerance);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("BangBangController");
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }

    @Override
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    @Override
    public void reset() {
    }
}
