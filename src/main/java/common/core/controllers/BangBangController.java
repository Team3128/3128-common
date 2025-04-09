package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;

public class BangBangController extends ControllerBase {

    public BangBangController(PIDFFConfig config) {
        super(config, 0.02);
    }

    @Override
    public void configureFeedback(NAR_Motor motor) {
        addMotor(motor);
    }

    @Override
    protected double calculatePID(double measurement) {
        return 0;
    }

    @Override
    protected double calculateFF(double pidOutput) {
        return !atSetpoint() ? Math.copySign(getConfig().getkS(), getError()) : 0;
    }

}
