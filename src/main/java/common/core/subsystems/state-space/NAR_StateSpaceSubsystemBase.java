package common.core.subsystems.statespace;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List; 

public class NAR_StateSpaceSubsystemBase<N extends Num> extends SubsystemBase {

    protected LinearSystemLoop<N, N1, N> loop;
    protected NAR_Motor motor;
    private Nat<N> stateSize;

    public NAR_StateSpaceSubsystemBase(LinearSystem<N, N1, N> system, Vector<N> stateSTD, Vector<N> measurementSTD, Vector<N> qelms, Vector<N1> relms, NAR_Motor motor) {
        KalmanFilter<N, N1, N> observer = new KalmanFilter<N, N1, N>(stateSize, stateSize, system, stateSTD, measurementSTD, 0.020);
        LinearQuadraticRegulator<N, N1, N> controller = new LinearQuadraticRegulator<N, N1, N>(system, qelms, relms, 0.020);
        this.loop = new LinearSystemLoop<>(system, controller, observer, 12.0, 0.020);
        this.motor = motor;
        this.stateSize = stateSize;
    }

    public NAR_StateSpaceSubsystemBase(LinearSystem<N1,N1,N1> identifyVelocitySystem, java.util.Vector<N1> stateSTD,
            java.util.Vector<N1> measurementSTD, java.util.Vector<N1> qelms, java.util.Vector<N1> relms,
            NAR_Motor motor2) {
        //TODO Auto-generated constructor stub
    }

    public void setStateSetpoint(double setpoint) {}

    public Vector<N> getMeasurement() {
        return new Vector<N>(stateSize);
    }

    @Override
    public void periodic() {
        loop.correct(getMeasurement());

        loop.predict(0.020);

        motor.setVolts(loop.getU(0));
    }
}