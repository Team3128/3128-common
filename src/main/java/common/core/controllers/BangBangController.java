package common.core.controllers;

import edu.wpi.first.util.sendable.SendableBuilder;

public class BangBangController extends Controller {

    private final edu.wpi.first.math.controller.BangBangController bangBang;

    public BangBangController(PIDFFConfig config, double tolerance) {
        super(config, tolerance, Type.VELOCITY);
        bangBang = new edu.wpi.first.math.controller.BangBangController(tolerance);
    }

    @Override
    protected double calculatePID(double measurement) {
        return bangBang.calculate(measurement);
    }

    @Override
    protected double calculateFF(double pidOutput) {
        if (atSetpoint()) {
            return config.getkS() + config.getkV() * getMeasurement();
        } else {
            return 0;
        }
    }

    @Override
    public void setSetpoint(double setpoint) {
        bangBang.setSetpoint(setpoint);
    }

    @Override
    public double getSetpoint() {
        return bangBang.getSetpoint();
    }

    @Override
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
        bangBang.setTolerance(tolerance);
    }

    @Override
    public boolean atSetpoint() {
        return bangBang.atSetpoint();
    }

    @Override
    public void reset() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("BangBangController");
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
