package common.core.subsystems;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
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
         * Full power below the target and none above, plus feedforward, using WPILib's
         * {@link BangBangController}. Recovers speed fastest after a game piece slows a flywheel
         * down. Only kV needs tuning: set it a little under the value that holds speed, so the
         * controller only ever adds power. The motors must be in coast mode, since braking fights
         * the controller. Slowing down to a lower setpoint is a slow coast.
         */
        BANG_BANG,
        /**
         * PID runs on the motor controller at 1 kHz instead of on the roboRIO at 50 Hz, e.g.
         * Phoenix {@code VelocityVoltage}. kS/kV feedforward is still in velocity units and is sent
         * along with the setpoint, but kP/kI/kD are in the motor controller's own units: volts per
         * motor rotation per second on a TalonFX, output fraction per motor RPM on a Spark.
         */
        MOTOR_CONTROLLER
    }

    private final VelocityControl control;

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
        control = null;
        motors[0].enableSim(new RollerSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, moiKgMetersSq));
    }

    /**
     * Creates a velocity-controlled roller.
     *
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Applied to every motor; sets the velocity units
     * @param control Where and how the velocity loop runs
     * @param gains kS/kV feedforward in volts per velocity unit, and kP/kI/kD for PID or MOTOR_CONTROLLER
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
            control == VelocityControl.BANG_BANG
                // BangBangController outputs 0 or 1, so scale it to full voltage
                ? (measurement, setpoint) -> MAX_VOLTS * bangBang.calculate(measurement, setpoint)
                : pid::calculate,
            measurement -> pid.reset(),
            () -> motors[0].getVelocity(), tolerance, 0, motorConfig, motors);
        setName(name);
        this.control = control;
        setDisableAtSetpoint(false); // keep spinning once at speed
        if (control == VelocityControl.MOTOR_CONTROLLER) {
            for (NAR_Motor motor : motors) {
                motor.configPID(gains);
            }
        }
        motors[0].enableSim(new RollerSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, moiKgMetersSq));
    }

    @Override
    protected double calculateFeedforward(double feedbackOutput) {
        return gains.velocityFF(getSetpoint(), feedbackOutput, atSetpoint());
    }

    @Override
    protected void applyOutput() {
        if (control != VelocityControl.MOTOR_CONTROLLER) {
            super.applyOutput();
            return;
        }
        // The error only signs kS; the motor controller does the PID
        final double feedforward = calculateFeedforward(getSetpoint() - getVelocity());
        for (NAR_Motor motor : motors) {
            motor.set(getSetpoint(), Control.Velocity, feedforward);
        }
    }
}
