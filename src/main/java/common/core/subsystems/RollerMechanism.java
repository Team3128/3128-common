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
 * A mechanism that spins: intakes, hoppers, kickers, shooter flywheels.
 *
 * <p>Every roller can run open loop with {@link #run(double)} or {@link #runVolts(double)}.
 * Rollers built with a {@link VelocityControl} can also hold a velocity with
 * {@link #setSetpoint(double)}, and continue spinning until disabled or run open loop.
 *
 * <pre>{@code
 * // An intake roller that is only turned on and off
 * intakeRoller = new RollerMechanism("IntakeRoller", ROLLER_CONFIG, DCMotor.getKrakenX60(1), 3,
 *     RollerSimModel.solidCylinderMoi(0.5, 0.025), new NAR_TalonFX(ROLLER_ID));
 * intakeRoller.run(0.8);
 *
 * // A flywheel that holds a speed, in rotations per second
 * flywheel = new RollerMechanism("Flywheel", FLYWHEEL_CONFIG, VelocityControl.PID, FLYWHEEL_GAINS, 1,
 *     DCMotor.getKrakenX60(1), 1, RollerSimModel.solidCylinderMoi(1.5, 0.05), new NAR_TalonFX(FLYWHEEL_ID));
 * flywheel.setSetpoint(60);
 * }</pre>
 */
public class RollerMechanism extends MechanismBase {

    /** How a velocity-controlled roller calculates its output. */
    public enum VelocityControl {
        /**
         * PID and feedforward on the motor controller at 1 kHz. Accelerates and decelerates
         * cleanly. Tune kV to hold speed, then kP to recover quickly after a game piece passes through.
         */
        PID,
        /**
         * Full power below the target and none above, plus feedforward, using WPILib's
         * {@link BangBangController} on the roboRIO. Only kV requires tuning: set it slightly below
         * the value that holds speed, so that the controller only ever adds power. The motors must be
         * in coast mode, since braking opposes the controller. Deceleration to a lower setpoint
         * occurs by coasting.
         */
        BANG_BANG
    }

    /** The velocity control method; null for an open-loop roller. */
    private final VelocityControl control;

    /**
     * Creates an open-loop roller.
     *
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Configuration applied to every motor
     * @param motorType Model of a single motor, for example, {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param gearing Motor rotations per roller rotation; used only in simulation
     * @param moiKgMetersSq Roller moment of inertia; see {@link RollerSimModel#solidCylinderMoi} and
     *                      {@link RollerSimModel#hollowCylinderMoi}; used only in simulation
     * @param motors Every motor on the roller. The first motor is the leader.
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
     * @param motorConfig Configuration applied to every motor; sets the velocity units
     * @param control PID or bang-bang
     * @param gains kS/kV feedforward, and kP/kI/kD for PID, all in volts per velocity unit
     * @param tolerance The distance from the setpoint within which {@link #atSetpoint()} is true, in velocity units
     * @param motorType Model of a single motor, for example, {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param gearing Motor rotations per roller rotation; used only in simulation
     * @param moiKgMetersSq Roller moment of inertia; used only in simulation
     * @param motors Every motor on the roller. The first motor is the leader.
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
        this.control = control;
        setDisableAtSetpoint(false);
        motors[0].enableSim(new RollerSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, moiKgMetersSq));
    }

    @Override
    protected double calculateFeedforward(double pidOutput) {
        // With PID, the motor controller applies the feedforward itself
        return control == VelocityControl.BANG_BANG ? gains.velocityFF(getSetpoint(), pidOutput, atSetpoint()) : 0;
    }
}
