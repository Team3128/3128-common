package common.core.controllers;
import edu.wpi.first.math.VecBuilder;
import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;


public class PositionSSController<N extends Num> extends ControllerBase {
    public NAR_Motor motor;
    public LinearSystemLoop<N2, N1, N2> loop;
    public double[] inputRange;
    public LinearQuadraticRegulator<N2, N1, N2> controller;
    public boolean is_continuous = false;
    
    public PositionSSController(PIDFFConfig config, Vector<N2> stateSTD, Vector<N2> measurementSTD, Vector<N2> qelms, Vector<N1> relms, double tolerance, double[] inputRange) {
        super(config, tolerance);
        this.inputRange = inputRange;

        LinearSystem<N2, N1, N2> system = LinearSystemId.identifyPositionSystem(config.getkV(), config.getkA());
        KalmanFilter<N2, N1, N2> observer = new KalmanFilter<N2, N1, N2>(Nat.N2(), Nat.N2(), system, stateSTD, measurementSTD, 0.020);
        controller = new LinearQuadraticRegulator<N2, N1, N2>(system, qelms, relms, 0.020);
        this.loop = new LinearSystemLoop<>(system, controller, observer, 12.0, 0.020);
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.loop.setNextR(VecBuilder.fill(MathUtil.clamp(setpoint, inputRange[0], inputRange[1]), 0));
    }

    @Override
    public void useOutput() {
        if (isEnabled() && atSetpoint()) disable();
        double measurement = getMeasurement();
        if (is_continuous) {
            double setpoint = this.loop.getNextR(0);
            double range = inputRange[1] - inputRange[0];
            if (Math.abs(setpoint - measurement) > range / 2) {
                if (setpoint > measurement) {
                    measurement += range;
                } else {
                    measurement -= range;
                }
            }
        }

        this.loop.correct(VecBuilder.fill(measurement, getVelocity()));
        this.loop.predict(0.020);
        final double output = MathUtil.clamp(this.loop.getU(0), -12, 12);
        for (NAR_Motor motor : getMotors()) {
            motor.setVolts(output);
        }
    }

    @Override
    public void setMeasurementSource(NAR_Motor m) {
        motor = m;
    }

    @Override
    public double getSetpoint() {
       return this.loop.getNextR(0);
    }

    @Override
    public double getMeasurement() {
        return motor.getPosition();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double[] getInputRange() {
        return inputRange;
    }

    @Override
    public boolean atSetpoint() {
        return VecBuilder.fill(getMeasurement(), getVelocity()).minus(getSetpoint()).normF() < tolerance;
    }

    //TODO: enableContinuousInput for position controller
    public void enableContinuousInput() {
        is_continuous = true;
    }

    @Override
    public void reset() {
        super.reset();
        this.loop.reset(VecBuilder.fill(0.0, 0.0));
    }

    @Override
    public void enable() {
        super.enable();
        reset();
    }
}
