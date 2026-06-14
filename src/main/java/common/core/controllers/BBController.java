package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.BangBangController;

public class BBController extends ControllerBase {

    BangBangController bbController;

    public BBController(PIDFFConfig config, double period) {
        super(config, period);
        bbController = new BangBangController();
    }

    public BBController(PIDFFConfig config) {
        this(config, 0.02);
    }

    public void setMeasurementSource(NAR_Motor motor) {
        setMeasurementSource(() -> motor.getVelocity());
    }

    public void configureFeedback(NAR_Motor motor) {
        setMeasurementSource(motor);
        addMotor(motor);
    }

    @Override
    public double calculatePID(double measurement) {
        return bbController.calculate(measurement, controller.getSetpoint());
    }

    @Override 
    public double calculateFF(double pidOutput) {
        if (atSetpoint()) {
            return getConfig().getkS() + getConfig().getkV() * getMeasurement();
        } else {
            return 0;
        }
    }

    @Override
    public boolean atSetpoint() {
        return bbController.atSetpoint();
    }
}
