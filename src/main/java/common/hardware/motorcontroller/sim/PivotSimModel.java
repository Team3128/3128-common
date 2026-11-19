package common.hardware.motorcontroller.sim;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * A mechanism that rotates to an angle between hard stops: an arm, wrist, hood, or turret. Angles
 * are in radians; with gravity, 0 is level.
 */
public class PivotSimModel implements MotorSimModel {

    /** Acceleration due to gravity, in m/s^2. */
    private static final double GRAVITY = 9.81;
    /** Gravity changes with the angle, so it is recalculated several times per robot loop. */
    private static final int STEPS_PER_UPDATE = 10;

    private final DCMotor motor;
    private final double gearing;
    private final double startRadians;
    private final boolean simulateGravity;
    /** Volts at the motor that would hold the mechanism level against gravity. */
    private final double levelGravityVolts;
    private final SingleJointedArmSim sim;
    private DoubleSupplier parentRadians = () -> 0;
    private DoubleSupplier extraGravityVolts = () -> 0;
    private double appliedVolts;

    /**
     * @param motor The motor(s) driving the mechanism
     * @param gearing Motor rotations per pivot rotation
     * @param lengthMeters Distance from the pivot to the far end of the moving part
     * @param massKg Mass of the moving part
     * @param minRadians Lower hard stop
     * @param maxRadians Upper hard stop
     * @param startRadians The angle when the robot turns on
     * @param simulateGravity True when the mechanism swings up and down, false for a turret
     */
    public PivotSimModel(DCMotor motor, double gearing, double lengthMeters, double massKg,
                         double minRadians, double maxRadians, double startRadians, boolean simulateGravity) {
        this.motor = motor;
        this.gearing = gearing;
        this.startRadians = startRadians;
        this.simulateGravity = simulateGravity;
        // SingleJointedArmSim's gravity knows only its own angle, so gravity is modeled here to
        // include a parent's: the weight acting at the middle of the moving part is a load torque,
        // which for a DC motor is equivalent to a back voltage.
        final double levelGravityTorque = massKg * GRAVITY * lengthMeters / 2 / gearing;
        this.levelGravityVolts = levelGravityTorque / motor.KtNMPerAmp * motor.rOhms;
        this.sim = new SingleJointedArmSim(motor, gearing, SingleJointedArmSim.estimateMOI(lengthMeters, massKg),
            lengthMeters, minRadians, maxRadians, false, startRadians);
    }

    /**
     * Mounts this model on another pivot that swings in the same plane, for example, a wrist on an arm, so that gravity
     * acts on the sum of their angles.
     * @param parentRadians The parent's angle above level
     */
    public void setParentAngle(DoubleSupplier parentRadians) {
        this.parentRadians = parentRadians;
    }

    /**
     * Adds a load from a mounted mechanism, for example, the extension of a telescope on an arm.
     * @param extraGravityVolts Volts at the motor that would hold the load
     */
    public void setExtraGravityVolts(DoubleSupplier extraGravityVolts) {
        this.extraGravityVolts = extraGravityVolts;
    }

    @Override public DCMotor getMotor() { return motor; }

    @Override
    public void update(double motorVolts, double dtSeconds) {
        appliedVolts = motorVolts;
        // Derived from other mechanisms' positions, which do not change until they are also updated
        final double extraVolts = extraGravityVolts.getAsDouble();
        for (int i = 0; i < STEPS_PER_UPDATE; i++) {
            final double gravityVolts = simulateGravity
                ? levelGravityVolts * Math.cos(sim.getAngleRads() + parentRadians.getAsDouble())
                : 0;
            sim.setInputVoltage(motorVolts - gravityVolts - extraVolts);
            sim.update(dtSeconds / STEPS_PER_UPDATE);
        }
    }

    @Override public double getRotorRotations() { return (sim.getAngleRads() - startRadians) / (2 * Math.PI) * gearing; }
    @Override public double getRotorRPS() { return sim.getVelocityRadPerSec() / (2 * Math.PI) * gearing; }

    @Override
    public double getCurrentAmps() {
        // Calculated from the real applied voltage, because the sim's input has gravity subtracted
        return motor.getCurrent(getRotorRPS() * 2 * Math.PI, appliedVolts);
    }
}
