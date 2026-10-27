package common.hardware.motorcontroller.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Something that moves in a straight line between hard stops, at any incline:
 * a slider (0 degrees), an elevator (90 degrees), or anything in between.
 */
public class LinearSimModel implements MotorSimModel {

    private static final double GRAVITY = 9.81; // m/s^2

    private final DCMotor motor;
    private final double motorRotationsPerMeter;
    private final double startMeters;
    private final double gravityVolts;
    private final ElevatorSim sim;
    private double appliedVolts;

    /**
     * @param motor The motor(s) driving the mechanism
     * @param motorRotationsPerMeter Motor rotations to move the mechanism one meter
     * @param massKg Mass that moves
     * @param minMeters Lower hard stop, also where the mechanism starts
     * @param maxMeters Upper hard stop
     * @param angleRadians Incline of the direction of travel: 0 is horizontal, pi/2 is straight up
     */
    public LinearSimModel(DCMotor motor, double motorRotationsPerMeter, double massKg,
                          double minMeters, double maxMeters, double angleRadians) {
        this.motor = motor;
        this.motorRotationsPerMeter = motorRotationsPerMeter;
        this.startMeters = minMeters;
        // ElevatorSim only depends on the ratio of gearing to drum radius, so model the drive
        // as a 1:1 drum whose circumference is the distance moved per motor rotation.
        final double drumRadiusMeters = 1.0 / (2 * Math.PI * motorRotationsPerMeter);
        // ElevatorSim's gravity is all or nothing, so we model gravity ourselves: the part of the
        // weight along the direction of travel is a constant load, which for a DC motor is the
        // same as a constant voltage pushing back.
        final double gravityTorque = massKg * GRAVITY * Math.sin(angleRadians) * drumRadiusMeters;
        this.gravityVolts = gravityTorque / motor.KtNMPerAmp * motor.rOhms;
        this.sim = new ElevatorSim(motor, 1, massKg, drumRadiusMeters, minMeters, maxMeters, false, minMeters);
    }

    @Override public DCMotor getMotor() { return motor; }

    @Override
    public void update(double motorVolts, double dtSeconds) {
        appliedVolts = motorVolts;
        sim.setInputVoltage(motorVolts - gravityVolts);
        sim.update(dtSeconds);
    }

    @Override public double getRotorRotations() { return (sim.getPositionMeters() - startMeters) * motorRotationsPerMeter; }
    @Override public double getRotorRPS() { return sim.getVelocityMetersPerSecond() * motorRotationsPerMeter; }

    @Override
    public double getCurrentAmps() {
        // Computed from the real applied voltage, since the sim's input has gravity subtracted
        return motor.getCurrent(getRotorRPS() * 2 * Math.PI, appliedVolts);
    }
}
