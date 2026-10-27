package common.hardware.motorcontroller.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Something that moves in a straight line between hard stops: an elevator or slider. */
public class LinearSimModel implements MotorSimModel {

    private final DCMotor motor;
    private final double motorRotationsPerMeter;
    private final double startMeters;
    private final ElevatorSim sim;

    /**
     * @param motor The motor(s) driving the mechanism
     * @param motorRotationsPerMeter Motor rotations to move the mechanism one meter
     * @param massKg Mass that moves
     * @param minMeters Lower hard stop, also where the mechanism starts
     * @param maxMeters Upper hard stop
     * @param simulateGravity True for a vertical elevator, false for a horizontal slider
     */
    public LinearSimModel(DCMotor motor, double motorRotationsPerMeter, double massKg,
                          double minMeters, double maxMeters, boolean simulateGravity) {
        this.motor = motor;
        this.motorRotationsPerMeter = motorRotationsPerMeter;
        this.startMeters = minMeters;
        // ElevatorSim only depends on the ratio of gearing to drum radius, so model the drive
        // as a 1:1 drum whose circumference is the distance moved per motor rotation.
        final double drumRadiusMeters = 1.0 / (2 * Math.PI * motorRotationsPerMeter);
        this.sim = new ElevatorSim(motor, 1, massKg, drumRadiusMeters, minMeters, maxMeters, simulateGravity, minMeters);
    }

    @Override public DCMotor getMotor() { return motor; }

    @Override
    public void update(double motorVolts, double dtSeconds) {
        sim.setInputVoltage(motorVolts);
        sim.update(dtSeconds);
    }

    @Override public double getRotorRotations() { return (sim.getPositionMeters() - startMeters) * motorRotationsPerMeter; }
    @Override public double getRotorRPS() { return sim.getVelocityMetersPerSecond() * motorRotationsPerMeter; }
    @Override public double getCurrentAmps() { return sim.getCurrentDrawAmps(); }
}
