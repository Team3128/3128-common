package common.core.controllers;
import edu.wpi.first.math.VecBuilder;
import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;


public class VelocitySSController<N extends Num> extends ControllerBase {
    public NAR_Motor motor;
    public LinearSystemLoop<N1, N1, N1> loop;
    
    public VelocitySSController(PIDFFConfig config, Vector<N1> stateSTD, Vector<N1> measurementSTD, Vector<N1> qelms, Vector<N1> relms, double tolerance) {
        super(config, tolerance);
        
        LinearSystem<N1, N1, N1> system = LinearSystemId.identifyVelocitySystem(config.getkV(), config.getkA());
        KalmanFilter<N1, N1, N1> observer = new KalmanFilter<N1, N1, N1>(Nat.N1(), Nat.N1(), system, stateSTD, measurementSTD, 0.020);
        LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<N1, N1, N1>(system, qelms, relms, 0.020);
        this.loop = new LinearSystemLoop<>(system, controller, observer, 12.0, 0.020);
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.loop.setNextR(VecBuilder.fill(setpoint));
    }

    @Override
    public void useOutput() {
        if (isEnabled() && atSetpoint()) disable();
        this.loop.correct(VecBuilder.fill(getMeasurement()));
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
        return motor.getVelocity();
    }

    @Override
    public boolean atSetpoint() {
        return VecBuilder.fill(getMeasurement()).minus(getSetpoint()).normF() < tolerance;
    }

    @Override
    public void reset() {
        super.reset();
        this.loop.reset(VecBuilder.fill(0.0));
    }

    @Override
    public void enable() {
        super.enable();
        reset();
    }
}
