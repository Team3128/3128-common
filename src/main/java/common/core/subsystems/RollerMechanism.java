package common.core.subsystems;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.sim.MotorSimModel;
import common.hardware.motorcontroller.sim.RollerSimModel;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Something that spins: intakes, hoppers, kickers, shooter flywheels.
 *
 * <p>Every roller can run open loop with {@link #run(double)} or {@link #runVolts(double)}.
 * Rollers built with a {@link VelocityControl} can also hold a velocity with
 * {@link #setSetpoint(double)}, and keep spinning until disabled or run open loop.
 */
public class RollerMechanism extends MechanismBase {

    /** How a velocity-controlled roller works out its output. */
    public enum VelocityControl {
        /** PID on the velocity error plus feedforward. Smooth, but kP needs tuning. */
        PID,
        /**
         * Full power below the target and none above, plus feedforward. Recovers speed fastest
         * after a game piece slows a flywheel down. Only kV needs tuning: set it a little under
         * the value that holds speed, so the controller only ever adds power.
         */
        BANG_BANG
    }

    private double overspeedTolerance = Double.POSITIVE_INFINITY;
    private double overspeedVolts = 0;

    /**
     * Creates an open-loop roller.
     *
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Applied to every motor
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param gearing Motor rotations per roller rotation; used only in simulation
     * @param moiKgMetersSq Roller inertia, see {@link RollerSimModel#solidCylinderMoi} and
     *                      {@link RollerSimModel#hollowCylinderMoi}; used only in simulation
     * @param motors Every motor on the roller. All of them get the same output.
     */
    public RollerMechanism(String name, MotorConfig motorConfig, DCMotor motorType, double gearing,
                           double moiKgMetersSq, NAR_Motor... motors) {
        super(motorConfig, motors);
        setName(name);
        motors[0].enableSim(new RollerSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, moiKgMetersSq));
    }

    /**
     * Creates a velocity-controlled roller.
     *
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Applied to every motor; sets the velocity units
     * @param control PID or bang-bang
     * @param gains kP/kI/kD (PID only) and kS/kV feedforward, in volts per velocity unit
     * @param tolerance How close to the setpoint counts as {@link #atSetpoint()}, in velocity units
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param gearing Motor rotations per roller rotation; used only in simulation
     * @param moiKgMetersSq Roller inertia; used only in simulation
     * @param motors Every motor on the roller. All of them get the same output.
     */
    public RollerMechanism(String name, MotorConfig motorConfig, VelocityControl control, PIDFFConfig gains,
                           double tolerance, DCMotor motorType, double gearing, double moiKgMetersSq,
                           NAR_Motor... motors) {
        this(name, motorConfig, control, gains, new PIDController(gains.kP, gains.kI, gains.kD),
            new BangBangController(), tolerance, motorType, gearing, moiKgMetersSq, motors);
    }

    private RollerMechanism(String name, MotorConfig motorConfig, VelocityControl control, PIDFFConfig gains,
                            PIDController pid, BangBangController bangBang, double tolerance, DCMotor motorType,
                            double gearing, double moiKgMetersSq, NAR_Motor... motors) {
        super(gains,
            control == VelocityControl.PID
                ? pid::calculate
                // BangBangController outputs 0 or 1, so scale it to full voltage
                : (measurement, setpoint) -> MAX_VOLTS * bangBang.calculate(measurement, setpoint),
            measurement -> pid.reset(),
            () -> motors[0].getVelocity(), tolerance, 0, motorConfig, motors);
        setName(name);
        setDisableAtSetpoint(false); // keep spinning once at speed
        motors[0].enableSim(new RollerSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, moiKgMetersSq));
    }

    /**
     * Applies {@code volts} whenever the roller is more than {@code toleranceOverTarget} above its
     * velocity setpoint, e.g. {@code setOverspeedBrake(6, -6)}. Bang-bang can't slow a roller down
     * on its own, so this helps it settle after the setpoint drops. Off by default.
     */
    public void setOverspeedBrake(double toleranceOverTarget, double volts) {
        overspeedTolerance = toleranceOverTarget;
        overspeedVolts = volts;
    }

    @Override
    protected double calculateFeedforward(double feedbackOutput) {
        return gains.velocityFF(getSetpoint(), feedbackOutput, atSetpoint());
    }

    @Override
    public void periodic() {
        if (isEnabled() && getVelocity() > getSetpoint() + overspeedTolerance) {
            for (NAR_Motor motor : motors) {
                motor.setVolts(overspeedVolts);
            }
            return;
        }
        super.periodic();
    }
}
