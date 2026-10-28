package common.hardware.motorcontroller.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Something that rotates to an angle between hard stops: an arm, hood, or turret. Angles are in
 * radians; with gravity, 0 is horizontal.
 */
public class PivotSimModel implements MotorSimModel {

    private final DCMotor motor;
    private final double gearing;
    private final double startRadians;
    private final SingleJointedArmSim sim;

    /**
     * @param motor The motor(s) driving it
     * @param gearing Motor rotations per pivot rotation
     * @param lengthMeters Pivot to the far end of the moving part
     * @param massKg Mass of the moving part
     * @param minRadians Lower hard stop
     * @param maxRadians Upper hard stop
     * @param startRadians Where it is when the robot turns on
     * @param simulateGravity True when it swings up and down, false for a turret
     */
    public PivotSimModel(DCMotor motor, double gearing, double lengthMeters, double massKg,
                         double minRadians, double maxRadians, double startRadians, boolean simulateGravity) {
        this.motor = motor;
        this.gearing = gearing;
        this.startRadians = startRadians;
        this.sim = new SingleJointedArmSim(motor, gearing, SingleJointedArmSim.estimateMOI(lengthMeters, massKg),
            lengthMeters, minRadians, maxRadians, simulateGravity, startRadians);
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
