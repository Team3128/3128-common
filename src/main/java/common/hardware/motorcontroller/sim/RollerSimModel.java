package common.hardware.motorcontroller.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Something that just spins: a roller, flywheel, or kicker. */
public class RollerSimModel implements MotorSimModel {

    private final DCMotor motor;
    private final double gearing;
    private final DCMotorSim sim;

    /**
     * @param motor The motor(s) driving the roller, e.g. {@code DCMotor.getKrakenX60(2)} for two motors
     * @param gearing Motor rotations per roller rotation
     * @param moiKgMetersSq Moment of inertia of the roller, see {@link #solidCylinderMoi(double, double)}
     */
    public RollerSimModel(DCMotor motor, double gearing, double moiKgMetersSq) {
        this.motor = motor;
        this.gearing = gearing;
        this.sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moiKgMetersSq, gearing), motor);
    }

    /**
     * Moment of inertia of a solid cylinder spinning about its axis.
     * @param massKg Mass in kilograms
     * @param radiusMeters Radius in meters
     * @return Inertia in kg*m^2
     */
    public static double solidCylinderMoi(double massKg, double radiusMeters) {
        return 0.5 * massKg * radiusMeters * radiusMeters;
    }

    @Override public DCMotor getMotor() { return motor; }

    @Override
    public void update(double motorVolts, double dtSeconds) {
        sim.setInputVoltage(motorVolts);
        sim.update(dtSeconds);
    }

    @Override public double getRotorRotations() { return sim.getAngularPositionRotations() * gearing; }
    @Override public double getRotorRPS() { return sim.getAngularVelocityRPM() / 60.0 * gearing; }
    @Override public double getCurrentAmps() { return sim.getCurrentDrawAmps(); }
}
