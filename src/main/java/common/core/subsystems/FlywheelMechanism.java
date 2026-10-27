package common.core.subsystems;

import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.sim.MotorSimModel;
import common.hardware.motorcontroller.sim.RollerSimModel;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Spins up to a target velocity and holds it: shooters.
 *
 * <p>Uses a bang-bang controller (full power below the target, none above) plus a feedforward
 * proportional to the measured speed. Optionally brakes with a fixed voltage when it is
 * well over the target, see {@link #setOverspeedBrake(double, double)}.
 */
public class FlywheelMechanism extends MechanismBase {

    private final BangBangController bangBang = new BangBangController();
    private final double kV;
    private final double tolerance;

    private double overspeedTolerance = Double.POSITIVE_INFINITY;
    private double overspeedVolts = 0;

    private boolean enabled = false;
    private double target = 0;

    /**
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Applied to every motor; sets the velocity units
     * @param kV Feedforward volts per unit of measured velocity
     * @param tolerance How close to the target counts as {@link #atSetpoint()}, in velocity units
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param gearing Motor rotations per flywheel rotation; used only in simulation
     * @param moiKgMetersSq Flywheel inertia, see {@link RollerSimModel#solidCylinderMoi}; used only in simulation
     * @param motors Every motor on the flywheel. All of them get the same output.
     */
    public FlywheelMechanism(String name, MotorConfig motorConfig, double kV, double tolerance,
                             DCMotor motorType, double gearing, double moiKgMetersSq, NAR_Motor... motors) {
        super(motorConfig, motors);
        setName(name);
        this.kV = kV;
        this.tolerance = tolerance;
        motors[0].enableSim(new RollerSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, moiKgMetersSq));
    }

    /**
     * Applies {@code volts} whenever the flywheel is more than {@code toleranceOverTarget} above
     * its target, e.g. {@code setOverspeedBrake(6, -6)}. Off by default.
     */
    public void setOverspeedBrake(double toleranceOverTarget, double volts) {
        overspeedTolerance = toleranceOverTarget;
        overspeedVolts = volts;
    }

    /** Spins up to {@code velocity}, or to the Shuffleboard debug setpoint when debug is on. */
    public void setTarget(double velocity) {
        enabled = true;
        target = (debug != null && debug.getAsBoolean()) ? debugSetpoint.getAsDouble() : velocity;
    }

    public Command setTargetCommand(double velocity) {
        return runOnce(() -> setTarget(velocity));
    }

    @Override
    public void periodic() {
        if (!enabled) return;
        final double velocity = getVelocity();
        final double volts = velocity > target + overspeedTolerance
            ? overspeedVolts
            : 12 * bangBang.calculate(velocity, target) + kV * velocity;
        for (NAR_Motor motor : motors) {
            motor.setVolts(volts);
        }
    }

    @Override public void setSetpoint(double setpoint) { setTarget(setpoint); }
    @Override public double getSetpoint() { return target; }
    @Override public void enable() { enabled = true; }
    @Override public boolean isEnabled() { return enabled; }

    /** Stops spinning. */
    @Override
    public void disable() {
        if (enabled) {
            for (NAR_Motor motor : motors) {
                motor.set(0);
            }
        }
        enabled = false;
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(getVelocity() - target) <= tolerance;
    }
}
