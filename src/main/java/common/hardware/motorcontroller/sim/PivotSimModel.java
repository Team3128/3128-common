package common.hardware.motorcontroller.sim;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Something that rotates to an angle between hard stops: an arm, wrist, hood, or turret. Angles
 * are in radians; with gravity, 0 is level.
 */
public class PivotSimModel implements MotorSimModel {

    private static final double GRAVITY = 9.81; // m/s^2
    // Gravity changes with the angle, so it's recalculated several times per robot loop
    private static final int STEPS_PER_UPDATE = 10;

    private final DCMotor motor;
    private final double gearing;
    private final double startRadians;
    private final boolean simulateGravity;
    /** Volts at the motor that would hold the mechanism level against gravity. */
    private final double levelGravityVolts;
    private final SingleJointedArmSim sim;
    private DoubleSupplier parentRadians = () -> 0;
    private double appliedVolts;

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
        this.simulateGravity = simulateGravity;
        // SingleJointedArmSim's gravity only knows its own angle, so we model gravity ourselves to
        // include a parent's: the weight acting at the middle of the moving part is a load torque,
        // which for a DC motor is the same as a voltage pushing back.
        final double levelGravityTorque = massKg * GRAVITY * lengthMeters / 2 / gearing;
        this.levelGravityVolts = levelGravityTorque / motor.KtNMPerAmp * motor.rOhms;
        this.sim = new SingleJointedArmSim(motor, gearing, SingleJointedArmSim.estimateMOI(lengthMeters, massKg),
            lengthMeters, minRadians, maxRadians, false, startRadians);
    }

    /**
     * Mounts this on another pivot that swings the same way, e.g. a wrist on an arm, so gravity
     * acts on the sum of their angles.
     * @param parentRadians The parent's angle above level
     */
    public void setParentAngle(DoubleSupplier parentRadians) {
        this.parentRadians = parentRadians;
    }

    @Override public DCMotor getMotor() { return motor; }

    @Override
    public void update(double motorVolts, double dtSeconds) {
        appliedVolts = motorVolts;
        for (int i = 0; i < STEPS_PER_UPDATE; i++) {
            final double gravityVolts = simulateGravity
                ? levelGravityVolts * Math.cos(sim.getAngleRads() + parentRadians.getAsDouble())
                : 0;
            sim.setInputVoltage(motorVolts - gravityVolts);
            sim.update(dtSeconds / STEPS_PER_UPDATE);
        }
    }

    @Override public double getRotorRotations() { return (sim.getAngleRads() - startRadians) / (2 * Math.PI) * gearing; }
    @Override public double getRotorRPS() { return sim.getVelocityRadPerSec() / (2 * Math.PI) * gearing; }

    @Override
    public double getCurrentAmps() {
        // Computed from the real applied voltage, since the sim's input has gravity subtracted
        return motor.getCurrent(getRotorRPS() * 2 * Math.PI, appliedVolts);
    }
}
