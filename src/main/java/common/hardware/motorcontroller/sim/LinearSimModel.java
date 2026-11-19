package common.hardware.motorcontroller.sim;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * A mechanism that moves in a straight line between hard stops, at any incline:
 * a slider (0 degrees), an elevator (90 degrees), or anything between.
 */
public class LinearSimModel implements MotorSimModel {

    /** Acceleration due to gravity, in m/s^2. */
    private static final double GRAVITY = 9.81;

    private final DCMotor motor;
    private final double motorRotationsPerMeter;
    private final double startMeters;
    /** Volts at the motor that would hold the mechanism pointing straight up against gravity. */
    private final double verticalGravityVolts;
    private DoubleSupplier inclineSine;
    private final ElevatorSim sim;
    private double appliedVolts;

    /**
     * @param motor The motor(s) driving the mechanism
     * @param motorRotationsPerMeter Motor rotations to move the mechanism one meter
     * @param massKg Moving mass
     * @param minMeters Lower hard stop, also the starting position
     * @param maxMeters Upper hard stop
     * @param angleRadians Incline of the direction of travel: 0 is horizontal, pi/2 is straight up
     */
    public LinearSimModel(DCMotor motor, double motorRotationsPerMeter, double massKg,
                          double minMeters, double maxMeters, double angleRadians) {
        this.motor = motor;
        this.motorRotationsPerMeter = motorRotationsPerMeter;
        this.startMeters = minMeters;
        // ElevatorSim depends only on the ratio of gearing to drum radius, so the drive is modeled
        // as a 1:1 drum whose circumference is the distance moved per motor rotation.
        final double drumRadiusMeters = 1.0 / (2 * Math.PI * motorRotationsPerMeter);
        // ElevatorSim's gravity is all or nothing, so gravity is modeled here: the portion of the
        // weight along the direction of travel is a load, which for a DC motor is equivalent to a
        // back voltage.
        final double verticalGravityTorque = massKg * GRAVITY * drumRadiusMeters;
        this.verticalGravityVolts = verticalGravityTorque / motor.KtNMPerAmp * motor.rOhms;
        final double sine = Math.sin(angleRadians);
        this.inclineSine = () -> sine;
        this.sim = new ElevatorSim(motor, 1, massKg, drumRadiusMeters, minMeters, maxMeters, false, minMeters);
    }

    /**
     * Sets the incline, for example, for a telescope on an arm.
     * @param inclineSine The sine of the current incline of travel
     */
    public void setInclineSine(DoubleSupplier inclineSine) {
        this.inclineSine = inclineSine;
    }

    @Override public DCMotor getMotor() { return motor; }

    @Override
    public void update(double motorVolts, double dtSeconds) {
        appliedVolts = motorVolts;
        sim.setInputVoltage(motorVolts - verticalGravityVolts * inclineSine.getAsDouble());
        sim.update(dtSeconds);
    }

    @Override public double getRotorRotations() { return (sim.getPositionMeters() - startMeters) * motorRotationsPerMeter; }
    @Override public double getRotorRPS() { return sim.getVelocityMetersPerSecond() * motorRotationsPerMeter; }

    @Override
    public double getCurrentAmps() {
        // Calculated from the real applied voltage, because the sim's input has gravity subtracted
        return motor.getCurrent(getRotorRPS() * 2 * Math.PI, appliedVolts);
    }
}
