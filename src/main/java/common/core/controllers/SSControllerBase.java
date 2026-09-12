package common.core.controllers;
import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.Matrix;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public abstract class SSControllerBase<N extends Num> implements Sendable {

    private final List<NAR_Motor> motors = new ArrayList<NAR_Motor>();
    protected SSConfig config;
    protected double tolerance;
    public SSController controller;
    public double setpoint;
    public boolean enabled = false;
    

        public class SSConfig {

        public DoubleSupplier kV;
        public DoubleSupplier kA;

        public Vector<N> stateSTD;
        public Nat<N> stateSize;
        public Vector<N> measurementSTD;

        public Vector<N> qelms;
        public Vector<N1> relms;

        public Nat<N> getStateSize() {
            return stateSize;
        }

        public double getkV() {
            return kV.getAsDouble();
        }

        public double getkA() {
            return kA.getAsDouble();
        }

        public Vector<N> getStateSTD() {
            return stateSTD;
        }

        public Vector<N> getMeasurementSTD() {
            return measurementSTD;
        }

        public Vector<N> getQelms() {
            return qelms;
        }

        public Vector<N1> getRelms() {
            return relms;
        }   
    }

    public SSControllerBase(LinearSystem<N, N1, N> system, SSConfig config, double tolerance) {
        this.config = config;
        this.tolerance = tolerance;
        
        this.controller = new SSController(system, config);


        requireNonNullParam(config, "config", "Controller");
    }

    public SSController getController() {
        return this.controller;
    }

    public class SSController {
        public LinearSystemLoop<N, N1, N> loop;
        public SSController(LinearSystem<N, N1, N> system, SSConfig config) {
            KalmanFilter<N, N1, N> observer = new KalmanFilter<N, N1, N>(getConfig().getStateSize(), getConfig().getStateSize(), system, getConfig().getStateSTD(), getConfig().getMeasurementSTD(), 0.020);
            
            // KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), system, VecBuilder.fill(config.getStateSTD()), Vector.fill(config.getMeasurementSTD()), 0.020);
            LinearQuadraticRegulator<N, N1, N> controller = new LinearQuadraticRegulator<N, N1, N>(system, getConfig().getQelms(), getConfig().getRelms(), 0.020);
            this.loop = new LinearSystemLoop<>(system, controller, observer, 12.0, 0.020);
        }

        public void reset() {
            this.loop.reset(new Matrix(VecBuilder.fill(0.0, 0.0)));
        }

        public void close() {
            
        }
    }



    public Vector<N> getMeasurement() {
        return new Vector<N>(config.getStateSize());
    }

    public void addMotor(NAR_Motor motor) {
        motors.add(motor);
    }

    public void setMeasurementSource(NAR_Motor motor) {}

    public void useOutput() {
        if (isEnabled() && atSetpoint()) disable();
        getController().loop.correct(getMeasurement());
        getController().loop.predict(0.020);
        final double output = MathUtil.clamp(getController().loop.getU(0), -12, 12);
        for (NAR_Motor motor : motors) {
            motor.setVolts(output);
        }
    }


    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public boolean atSetpoint() {
        return getMeasurement().minus(getSetpoint()).normF() < tolerance;
    }

    public Matrix<N, N1> getSetpoint() {
        Matrix <N, N1> rMatrix =  this.controller.loop.getNextR();
        return new Vector<>(rMatrix.getStorage());
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint) {}


    /** Resets the previous error and the integral term. */
    public void reset() {
        for (NAR_Motor motor : motors) {
            motor.setVolts(0);
        }
    }

    public SSConfig getConfig() {
        return this.config;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        // builder.addDoubleProperty("p", config::getkP, config::setkP);
        // builder.addDoubleProperty("i", config::getkI, config::setkI);
        // builder.addDoubleProperty("d", config::getkD, config::setkD);
        // builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
