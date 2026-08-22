package common.core.controllers;
import common.hardware.motorcontroller.NAR_Motor;

public class SSController extends ControllerBase {

    public SSController(PIDFFConfig config, double period) {
        super(config, period);
    }

    @Override
    public double calculatePID(double measurement) {
        return 0;
    }

    @Override
    public double calculateFF(double pidOutput) {
        return 0;
    }

    @Override
    public void configureFeedback(NAR_Motor motor) {
        setMeasurementSource(motor);
        addMotor(motor);
    }

    public void setMeasurementSource(NAR_Motor motor) {
        if (type == Type.VELOCITY) {
            setMeasurementSource(()-> motor.getVelocity());
            return;
        }
        setMeasurementSource(()-> motor.getPosition());
    }



    @Override
    public double useOutput() {
        if(isEnabled() && atSetpoint() && disabledAtSetpoint) disable();
        if (measurement == null || !isEnabled()) return 0;
        final double output = MathUtil.clamp(calculate(getMeasurement()), outputMin, outputMax);
        for (final DoubleConsumer consumer : consumers) {
            consumer.accept(output);
        }
        return output;
    }

}
