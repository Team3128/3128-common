package common.hardware.motorcontroller.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** A pivoting arm with gravity and hard stops. Angles are in radians, 0 is horizontal. */
public class ArmSimModel implements MotorSimModel {

    private final DCMotor motor;
    private final double gearing;
    private final double startRadians;
    private final SingleJointedArmSim sim;

    /**
     * @param motor The motor(s) driving the arm
     * @param gearing Motor rotations per arm rotation
     * @param lengthMeters Pivot to the end of the arm
     * @param massKg Mass of the arm
     * @param minRadians Lower hard stop
     * @param maxRadians Upper hard stop
     * @param startRadians Where the arm is when the robot turns on
     */
    public ArmSimModel(DCMotor motor, double gearing, double lengthMeters, double massKg,
                       double minRadians, double maxRadians, double startRadians) {
        this.motor = motor;
        this.gearing = gearing;
        this.startRadians = startRadians;
        this.sim = new SingleJointedArmSim(motor, gearing, SingleJointedArmSim.estimateMOI(lengthMeters, massKg),
            lengthMeters, minRadians, maxRadians, true, startRadians);
    }

    @Override public DCMotor getMotor() { return motor; }

    @Override
    public void update(double motorVolts, double dtSeconds) {
        sim.setInputVoltage(motorVolts);
        sim.update(dtSeconds);
    }

    @Override public double getRotorRotations() { return (sim.getAngleRads() - startRadians) / (2 * Math.PI) * gearing; }
    @Override public double getRotorRPS() { return sim.getVelocityRadPerSec() / (2 * Math.PI) * gearing; }
    @Override public double getCurrentAmps() { return sim.getCurrentDrawAmps(); }
}
