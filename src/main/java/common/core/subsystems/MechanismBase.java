package common.core.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.NAR_SysIdCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Base class for a mechanism that drives its motors to a setpoint.
 *
 * <p>The control loop can run in one of two places:
 * <ul>
 * <li><b>On the motor controller</b> (the usual choice): the TalonFX or Spark runs PID at 1 kHz
 *     on its own encoder, e.g. Phoenix {@code PositionVoltage} or {@code VelocityVoltage}.
 *     Gains are given in this mechanism's units and converted for the motor controller.
 *     <pre>{@code super(gains, Control.Position, TOLERANCE, HOME, MOTOR_CONFIG, motor);}</pre></li>
 * <li><b>On the roboRIO</b>, with a WPILib controller such as a {@code BangBangController},
 *     or when the measurement doesn't come from the motor's encoder, e.g. a gyro or camera.
 *     The controller is a pure {@code (measurement, setpoint) -> volts} calculator:
 *     <pre>{@code super(gains, pid::calculate, m -> pid.reset(), gyro::getAngle, TOLERANCE, HOME, MOTOR_CONFIG, motor);}</pre></li>
 * </ul>
 *
 * <p>On the motor controller, feedforward runs there too: kS, kV, and kG either constant or
 * times the cosine of an arm's angle (see {@link #useArmGravity(double)}). On the roboRIO,
 * subclasses work it out in {@link #calculateFeedforward(double)}.
 *
 * <p>When several motors drive one mechanism, the first is the leader and the rest follow it
 * on the motor controller where possible. Only the leader runs the control loop.
 */
public abstract class MechanismBase extends SubsystemBase {

    /** Nominal battery voltage. Closed-loop output is limited to plus or minus this. */
    public static final double MAX_VOLTS = 12;

    protected final NAR_Motor[] motors;
    /** The leader, plus any motor that can't follow it on the motor controller. */
    private final NAR_Motor[] commandedMotors;
    protected final PIDFFConfig gains;

    /** Position or Velocity when the motor controller runs the loop; null when the roboRIO does. */
    private final Control controllerMode;
    /** Gearing to an arm when kG follows the arm's cosine; 0 when kG is constant. */
    private double motorRotationsPerArmRotation = 0;
    /** Computes controller output from (measurement, setpoint), e.g. {@code pid::calculate}. */
    private final DoubleBinaryOperator feedback;
    /** Resets the controller's internal state, given the current measurement. */
    private final DoubleConsumer resetController;
    private final DoubleSupplier measurement;
    private final double tolerance;
    private final double resetPosition;

    private double setpointValue;
    private double inputMin = Double.NEGATIVE_INFINITY;
    private double inputMax = Double.POSITIVE_INFINITY;
    private boolean enabled = false;
    private boolean disableAtSetpoint = true;

    private double safetyThresh = 5;
    private final Timer safetyTimer = new Timer();
    private MotorConfig motorConfig;

    protected BooleanSupplier debug;
    protected DoubleSupplier debugSetpoint;

    /**
     * Creates a mechanism whose PID runs on the leader's motor controller.
     *
     * @param gains kP/kI/kD in volts per unit of error, kS and kG in volts, kV in volts per unit of
     *              velocity; converted for the motor controller.
     * @param controllerMode {@link Control#Position} or {@link Control#Velocity}.
     * @param tolerance Error tolerance for {@link #atSetpoint()}.
     * @param resetPosition Position the motors are reset to by {@link #reset()}, e.g. the mechanism's home position.
     * @param motorConfig Hardware configuration applied to every motor; sets the units.
     * @param motors The motor(s) driven by this mechanism. The first is the leader.
     */
    public MechanismBase(PIDFFConfig gains, Control controllerMode, double tolerance, double resetPosition,
                          MotorConfig motorConfig, NAR_Motor... motors) {
        this(gains, controllerMode, null, null,
            controllerMode == Control.Velocity ? () -> motors[0].getVelocity() : () -> motors[0].getPosition(),
            tolerance, resetPosition, motorConfig, motors);
    }

    /**
     * Creates a mechanism whose control loop runs on the roboRIO.
     *
     * @param gains Feedforward gains; also handed to {@link #calculateFeedforward(double)}.
     * @param feedback The WPILib controller's {@code calculate(measurement, setpoint)} method reference.
     * @param resetController Resets the controller's internal state (integral, previous error, profile) given the
     *                        current measurement; called when the mechanism goes from disabled to enabled. Use
     *                        {@code m -> pid.reset()} for a {@code PIDController}, {@code profiled::reset} for a
     *                        {@code ProfiledPIDController}, or {@code m -> {}} for a {@code BangBangController}.
     * @param measurement Supplies the mechanism's current measurement, e.g. {@code leader::getPosition}.
     * @param tolerance Error tolerance for {@link #atSetpoint()}.
     * @param resetPosition Position the motors are reset to by {@link #reset()}, e.g. the mechanism's home position.
     * @param motorConfig Hardware configuration applied to every motor.
     * @param motors The motor(s) driven by this mechanism. The first is the leader.
     */
    public MechanismBase(PIDFFConfig gains, DoubleBinaryOperator feedback, DoubleConsumer resetController,
                          DoubleSupplier measurement, double tolerance, double resetPosition,
                          MotorConfig motorConfig, NAR_Motor... motors) {
        this(gains, null, feedback, resetController, measurement, tolerance, resetPosition, motorConfig, motors);
    }

    /**
     * Takes either a {@code controllerMode} for a loop on the motor controller, or a
     * {@code feedback} for a loop on the roboRIO; leave the other null. Both null is open loop.
     */
    protected MechanismBase(PIDFFConfig gains, Control controllerMode, DoubleBinaryOperator feedback,
                            DoubleConsumer resetController, DoubleSupplier measurement, double tolerance,
                            double resetPosition, MotorConfig motorConfig, NAR_Motor... motors) {
        requireNonNullParam(motors, "motors", "MechanismBase");
        requireNonNullParam(gains, "gains", "MechanismBase");
        if (motors.length == 0) {
            throw new IllegalArgumentException("MechanismBase requires at least one motor");
        }
        for (int i = 0; i < motors.length; i++) {
            requireNonNullParam(motors[i], "motors[" + i + "]", "MechanismBase");
        }

        this.gains = gains;
        this.controllerMode = controllerMode;
        this.feedback = feedback;
        this.resetController = resetController;
        this.measurement = measurement;
        this.tolerance = tolerance;
        this.resetPosition = resetPosition;
        this.motorConfig = motorConfig;
        this.motors = motors;

        for (NAR_Motor motor : motors) {
            motor.configMotor(motorConfig);
        }

        final List<NAR_Motor> commanded = new ArrayList<>();
        commanded.add(motors[0]);
        for (int i = 1; i < motors.length; i++) {
            if (!motors[i].followOnController(motors[0], false)) commanded.add(motors[i]);
        }
        commandedMotors = commanded.toArray(new NAR_Motor[0]);

        applyGains();
    }

    /**
     * Sends the PID and feedforward gains to the motor controllers. Call again after changing
     * {@code gains}, e.g. from Shuffleboard. Does nothing when the roboRIO runs the loop.
     */
    public void applyGains() {
        if (controllerMode == null) return;
        for (NAR_Motor motor : commandedMotors) {
            if (motorRotationsPerArmRotation > 0) motor.configArmClosedLoop(gains, motorRotationsPerArmRotation);
            else motor.configClosedLoop(gains, controllerMode);
        }
    }

    /**
     * Makes the motor controller multiply kG by the cosine of the arm's angle instead of applying
     * it as a constant. Position 0 must be the arm held horizontal.
     * @param motorRotationsPerArmRotation Rotations of the leader's encoder per arm rotation: the
     *                                     gearing, or 1 for an absolute encoder on the arm's shaft
     */
    protected void useArmGravity(double motorRotationsPerArmRotation) {
        this.motorRotationsPerArmRotation = motorRotationsPerArmRotation;
        applyGains();
    }

    /**
     * Creates an open-loop-only mechanism, with no PID/feedforward loop.
     * {@link #enable()}/{@link #setSetpoint(double)} are unavailable; use {@link #run(double)}
     * or {@link #runVolts(double)} directly.
     * The measurement defaults to the first motor's position so Shuffleboard widgets still work.
     */
    public MechanismBase(MotorConfig motorConfig, NAR_Motor... motors) {
        this(new PIDFFConfig(), null, null, () -> motors[0].getPosition(), 0, 0, motorConfig, motors);
    }

    /** Makes one motor spin the opposite way to {@code motorConfig}, e.g. when it faces the other way. */
    public void invertMotor(int motorIndex) {
        final NAR_Motor motor = motors[motorIndex];
        // A follower's direction is set relative to the leader rather than by its own inversion
        if (motorIndex == 0 || !motor.followOnController(motors[0], true)) {
            motor.setInverted(!motorConfig.inverted);
        }
    }

    /** Sets the stator current limit of every motor, e.g. to push gently against a hard stop. */
    public void setStatorLimit(int amps) {
        for (NAR_Motor motor : motors) {
            motor.setStatorLimit(amps);
        }
    }

    /**
     * Feedforward worked out on the roboRIO, in volts. Default is none.
     *
     * <p>When the roboRIO runs the loop, override this to add e.g.
     * {@code gains.positionFF(pidOutput, atSetpoint())}. When the motor controller runs the loop
     * it already applies kS, kV and kG, so only override this for anything extra it can't do,
     * e.g. a kG that changes with an elevator's height.
     *
     * @param pidOutput The output of {@code feedback} this cycle, or the error (setpoint minus
     *                  measurement) when the motor controller runs the loop. Use it to sign kS.
     */
    protected double calculateFeedforward(double pidOutput) {
        return 0;
    }

    @Override
    public void periodic() {
        if (enabled) {
            if (controllerMode != null) {
                // The motor controller runs PID and feedforward; this only adds anything extra
                final double feedforward = calculateFeedforward(setpointValue - measurement.getAsDouble());
                for (NAR_Motor motor : commandedMotors) {
                    motor.set(setpointValue, controllerMode, feedforward);
                }
            } else {
                final double pidOutput = feedback.applyAsDouble(measurement.getAsDouble(), setpointValue);
                final double output = MathUtil.clamp(pidOutput + calculateFeedforward(pidOutput), -MAX_VOLTS, MAX_VOLTS);
                for (NAR_Motor motor : commandedMotors) {
                    motor.setVolts(output);
                }
            }

            if (safetyTimer.hasElapsed(safetyThresh)) onSafetyTimeout();
            if (atSetpoint()) {
                safetyTimer.restart();
                NAR_Shuffleboard.addData(getName(), "AtSetpoint", true, 1, 0);
                if (disableAtSetpoint) disable();
            }
        }

        NAR_Shuffleboard.addData(getName(), "Velocity", motors[0].getVelocity(), 5, 1);
    }

    /**
     * Returns the MotorConfig object controlling the subsystem
     *
     * @return The MotorConfig
     */
    public MotorConfig getMotorConfig() {
        return motorConfig;
    }

    /**
     * Sets the safetyThreshold to disable PID if setpoint is not reached
     * @param timeSeconds The time in seconds for the safety threshold
     */
    public void setSafetyThresh(double timeSeconds) {
        safetyThresh = timeSeconds;
    }

    /**
     * Called when the safety timeout is reached.
     * Disables the PID control and stops the motors.
     */
    public void onSafetyTimeout(){
        Log.unusual(getName(), "Safety Timeout Reached");
        disable();
    }

    /**
     * Sets the setpoint for the subsystem.
     *
     * @param setpoint the setpoint for the subsystem
     */
    public void setSetpoint(double setpoint) {
        enable();
        final double requested = (debug != null && debug.getAsBoolean()) ? debugSetpoint.getAsDouble() : setpoint;
        setpointValue = MathUtil.clamp(requested, inputMin, inputMax);
        NAR_Shuffleboard.addData(getName(), "AtSetpoint", false, 1, 0);
    }

    public Command setSetpointCommand(double setpoint) {
        return runOnce(() -> setSetpoint(setpoint));
    }

    /**
     * Sets whether PID control disables itself (stopping the motors) once
     * {@link #atSetpoint()} is true. Defaults to true; set to false for mechanisms that must keep
     * holding position, e.g. an elevator.
     */
    public void setDisableAtSetpoint(boolean disableAtSetpoint) {
        this.disableAtSetpoint = disableAtSetpoint;
    }

    /**
     * Limits every setpoint passed to {@link #setSetpoint(double)} to {@code [min, max]}.
     * Unlimited by default.
     */
    public void setInputRange(double min, double max) {
        inputMin = min;
        inputMax = max;
    }

    /**
     * Returns the current setpoint of the subsystem.
     *
     * @return The current setpoint
     */
    public double getSetpoint() {
        return setpointValue;
    }

    /**
     * Returns true if subsystem is at setpoint, false if not
     *
     * @return If subsystem is at setpoint
     */
    public boolean atSetpoint() {
        return Math.abs(measurement.getAsDouble() - setpointValue) < tolerance;
    }

    /** Enables the PID control. Resets the controller if it was previously disabled. */
    public void enable() {
        if (controllerMode == null) {
            requireNonNullParam(feedback, "feedback", "MechanismBase.enable");
            if (!enabled) resetController.accept(measurement.getAsDouble());
        }
        enabled = true;
        safetyTimer.restart();
        Log.debug(Log.Type.CONTROLLER, getName(), "Enabled PID");
    }

    /** Disables the PID control. If it was running, also stops the motors so they don't hold their last output. */
    public void disable() {
        if (enabled) {
            for (NAR_Motor motor : commandedMotors) {
                motor.set(0);
            }
        }
        enabled = false;
        Log.debug(Log.Type.CONTROLLER, getName(), "Disabled PID");
    }

    /**
     * Returns whether the controller is enabled.
     *
     * @return Whether the controller is enabled.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Sets power to motors. Disables PID control.
     *
     * @param power The power to set the motors to between -1 and 1.
     */
    public void run(double power) {
        disable();
        for (NAR_Motor motor : commandedMotors) {
            motor.set(power);
        }
    }

    /**
     * Sets power to motors.
     *
     * @param power The power to set the motors to between -1 and 1.
     * @return Command to run power
     */
    public Command runCommand(double power) {
        return runOnce(() -> run(power));
    }

    /**
     * Sets voltage to motors. Disables PID control.
     *
     * @param volts The voltage to set the motors to.
     */
    public void runVolts(double volts) {
        disable();
        for (NAR_Motor motor : commandedMotors) {
            motor.setVolts(volts);
        }
    }

    /**
     * Sets voltage to motors.
     *
     * @param volts The voltage to set the motors to.
     */
    public Command runVoltsCommand(double volts) {
        return runOnce(() -> runVolts(volts));
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        disable();
        run(0);
    }

    /**
     * Resets every motor's position to the {@code resetPosition} given at construction.
     */
    public void reset() {
        reset(resetPosition);
    }

    /**
     * Resets every motor's position to the {@code resetPosition} given at construction.
     */
    public Command resetCommand() {
        return runOnce(this::reset);
    }

    /**
     * Resets every motor's position to {@code position}.
     */
    public void reset(double position) {
        disable();
        for (NAR_Motor motor : motors) {
            motor.resetPosition(position);
        }
    }

    /**
     * Resets every motor's position to {@code position}.
     */
    public Command resetCommand(double position) {
        return runOnce(() -> reset(position));
    }

    /**
     * Get the position of the mechanism relative to its reset.
     *
     * @return The position of the first motor.
     */
    public double getPosition() {
        return motors[0].getPosition();
    }

    /**
     * Get the velocity of the mechanism.
     *
     * @return The velocity of the first motor.
     */
    public double getVelocity() {
        return motors[0].getVelocity();
    }

    /**
     * Get the volts applied to the mechanism
     *
     * @return The volts applied to the first motor.
     */
    public double getVolts() {
        return motors[0].getAppliedOutput() * MAX_VOLTS;
    }

    public Command characterization(double rampRate, double stepVoltage) {
       NAR_SysIdCommand characterize = new NAR_SysIdCommand(rampRate, stepVoltage, (v) -> runVolts(v.in(Volts)), this, motors[0]);
       return characterize.runSysId();
    }

    /**
     * Adds the underlying WPILib controller's own Shuffleboard widget (it already implements
     * {@link Sendable}), e.g. {@code addControllerWidget(pid)}.
     */
    protected void addControllerWidget(Sendable controller, int x, int y, int width, int height) {
        NAR_Shuffleboard.addSendable(getName(), "PID_Controller", controller, x, y, width, height).withWidget(BuiltInWidgets.kPIDController);
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData(getName(), "Enabled", this::isEnabled, 0, 0);
        NAR_Shuffleboard.addData(getName(), "AtSetpoint", this::atSetpoint, 1, 0);
        NAR_Shuffleboard.addData(getName(), "Measurement", measurement::getAsDouble, 0, 1);
        NAR_Shuffleboard.addData(getName(), "Setpoint", this::getSetpoint, 1, 1);

        debug = NAR_Shuffleboard.debugSwitch(getName(), "DEBUG", false, 2, 0);
        debugSetpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 2, 1);

        NAR_Shuffleboard.addData(getName(), "Measurement Graph", measurement::getAsDouble, 6, 0, 2, 2).withWidget(BuiltInWidgets.kGraph);
        NAR_Shuffleboard.addData(getName(), "Setpoint Graph", this::getSetpoint, 8, 0, 2, 2).withWidget(BuiltInWidgets.kGraph);

        FFWidgets(1, 0);
        runVoltsWidgets(debug, 1, 0);

        NAR_Shuffleboard.addCommand(getName(), "Enable", either(startEnd(() -> setSetpoint(debugSetpoint.getAsDouble()), this::disable), print("DEBUG NOT ON"), debug), 4, 0);
    }

    private void runVoltsWidgets(BooleanSupplier debug, int x, int y) {
        final DoubleSupplier debugVoltage = NAR_Shuffleboard.debug(getName(), "Debug Volts", 0, x + 6, y + 3);
        NAR_Shuffleboard.addCommand(getName(), "Run Volts", either(startEnd(() -> runVolts(debugVoltage.getAsDouble()), this::stop), print("DEBUG NOT ON"), debug), x + 7, y + 3);
        NAR_Shuffleboard.addData(getName(), "Running", () -> debug.getAsBoolean() && getVolts() > 0, x + 5, y + 3);
        NAR_Shuffleboard.addData(getName(), "Voltage", this::getVolts, x + 4, y + 3);
    }

    private void FFWidgets(int x, int y) {
        gains.setkS(NAR_Shuffleboard.debug(getName(), "kS", gains.getkS(), x, y + 2));
        gains.setkV(NAR_Shuffleboard.debug(getName(), "kV", gains.getkV(), x + 1, y + 2));
        gains.setkA(NAR_Shuffleboard.debug(getName(), "kA", gains.getkA(), x + 1, y + 3));
        gains.setkG(NAR_Shuffleboard.debug(getName(), "kG", gains.getkG(), x, y + 3));
        NAR_Shuffleboard.addCommand(getName(), "Characterize", characterization(1, 0.5), x - 1, y + 3).withSize(1, 1);
        // The motor controller only sees new gains once they're sent to it
        if (controllerMode != null) {
            NAR_Shuffleboard.addCommand(getName(), "Apply Gains", runOnce(this::applyGains).ignoringDisable(true), x + 2, y + 2);
        }
    }
}
