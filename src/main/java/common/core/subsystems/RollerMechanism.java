package common.core.subsystems;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.sim.MotorSimModel;
import common.hardware.motorcontroller.sim.RollerSimModel;
import edu.wpi.first.math.controller.BangBangController;
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
        /**
         * PID on the motor controller at 1 kHz, plus feedforward. Speeds up and slows down
         * cleanly. Tune kV to hold speed, then kP to recover quickly after a game piece.
         */
        PID,
        /**
         * Full power below the target and none above, plus feedforward, using WPILib's
         * {@link BangBangController} on the roboRIO. Only kV needs tuning: set it a little under
         * the value that holds speed, so the controller only ever adds power. The motors must be
         * in coast mode, since braking fights the controller. Slowing down to a lower setpoint
         * is a slow coast.
         */
        BANG_BANG
    }

    /**
     * Creates an open-loop roller.
     *
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Applied to every motor
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param gearing Motor rotations per roller rotation; used only in simulation
     * @param moiKgMetersSq Roller inertia, see {@link RollerSimModel#solidCylinderMoi} and
     *                      {@link RollerSimModel#hollowCylinderMoi}; used only in simulation
     * @param motors Every motor on the roller. The first is the leader.
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
     * @param gains kS/kV feedforward, and kP/kI/kD for PID, all in volts per velocity unit
     * @param tolerance How close to the setpoint counts as {@link #atSetpoint()}, in velocity units
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param gearing Motor rotations per roller rotation; used only in simulation
     * @param moiKgMetersSq Roller inertia; used only in simulation
     * @param motors Every motor on the roller. The first is the leader.
     */
    public RollerMechanism(String name, MotorConfig motorConfig, VelocityControl control, PIDFFConfig gains,
                           double tolerance, DCMotor motorType, double gearing, double moiKgMetersSq,
                           NAR_Motor... motors) {
        this(name, motorConfig, control, gains, new BangBangController(), tolerance, motorType, gearing,
            moiKgMetersSq, motors);
    }

    private RollerMechanism(String name, MotorConfig motorConfig, VelocityControl control, PIDFFConfig gains,
                            BangBangController bangBang, double tolerance, DCMotor motorType, double gearing,
                            double moiKgMetersSq, NAR_Motor... motors) {
        super(gains,
            control == VelocityControl.PID ? Control.Velocity : null,
            // BangBangController outputs 0 or 1, so scale it to full voltage
            control == VelocityControl.BANG_BANG
                ? (measurement, setpoint) -> MAX_VOLTS * bangBang.calculate(measurement, setpoint)
                : null,
            measurement -> {},
            () -> motors[0].getVelocity(), tolerance, 0, motorConfig, motors);
        setName(name);
        setDisableAtSetpoint(false); // keep spinning once at speed
        motors[0].enableSim(new RollerSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, moiKgMetersSq));
    }

    @Override
    protected double calculateFeedforward(double pidOutput) {
        return gains.velocityFF(getSetpoint(), pidOutput, atSetpoint());
    }
}
