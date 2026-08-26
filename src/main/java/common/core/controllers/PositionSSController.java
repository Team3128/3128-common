package common.core.controllers;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;


public class PositionSSController<N extends Num> extends ControllerBase {
    public class SSConfig {

        public DoubleSupplier kV;
        public DoubleSupplier kA;

        public double stateSTD;
        public double measurementSTD;

        public double qelms;
        public double relms;

        public double getkV() {
            return kV.getAsDouble();
        }

        public double getkA() {
            return kA.getAsDouble();
        }

        public double getStateSTD() {
            return stateSTD;
        }

        public double getMeasurementSTD() {
            return measurementSTD;
        }

        public double getQelms() {
            return qelms;
        }

        public double getRelms() {
            return relms;
        }

        
        
    }

    public PositionSSController(SSConfig config, double tolerance) {
        super(config, tolerance);
        
        LinearSystem<N1, N1, N1> system = LinearSystemId.identifyVelocitySystem(config.getkV(), config.getkA());

                
        KalmanFilter<N, N1, N> observer = new KalmanFilter<N, N1, N>(Nat.N1(), Nat.N1(), system, VecBuilder.fill(config.getStateSTD()), VecBuilder.fill(config.getMeasurementSTD()), 0.020);
        LinearQuadraticRegulator<N, N1, N> controller = new LinearQuadraticRegulator<N, N1, N>(system, VecBuilder.fill(config.getQelms()), VecBuilder.fill(config.getRelms()), 0.020);
        this.loop = new LinearSystemLoop<>(system, controller, observer, 12.0, 0.020);
        this.motor = motor;
    }
}
