package common.core.subsystems;

import java.util.ArrayList;
import java.util.Collections;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Base class for mechanisms: one or more motors driven to a setpoint, with logging, Shuffleboard
 * tuning widgets and a safety timeout.
 *
 * <p>Most robot code uses a subclass: {@link LinearMechanism}, {@link PivotMechanism} or
 * {@link RollerMechanism}. Extend this class directly only for a mechanism those do not cover.
 * See {@code README.md} in this package for how control loops, followers and logging work.
 */
public abstract class MechanismBase extends SubsystemBase {

    /** Nominal battery voltage. Closed-loop output is limited to this magnitude. */
    public static final double MAX_VOLTS = 12;

    protected final NAR_Motor[] motors;
    /** The leader, plus any motor that cannot follow it on the motor controller. */
    private final NAR_Motor[] commandedMotors;
    protected final PIDFFConfig gains;

    /** Position or Velocity when the motor controller runs the loop; null when the roboRIO does. */
    private final Control controllerMode;
    /** Gearing to an arm when kG follows the arm's cosine; 0 when kG is constant. */
    private double motorRotationsPerArmRotation = 0;
    /** False when kG is calculated on the roboRIO instead, in {@link #calculateFeedforward(double)}. */
    private boolean kGOnController = true;
    /** Computes controller output from (measurement, setpoint), for example, {@code pid::calculate}. */
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
    private MechanismLog log;
    /** The mechanism on which this one is mounted, for the 3D view; null if mounted on the robot. */
    private MechanismBase parent;
    private final List<MechanismBase> mounted = new ArrayList<>();

    protected BooleanSupplier debug;
    protected DoubleSupplier debugSetpoint;

    /**
     * Creates a mechanism whose PID runs on the leader's motor controller.
     *
     * @param gains kP/kI/kD in volts per unit of error, kS and kG in volts, kV in volts per unit of
     *              velocity; converted for the motor controller.
     * @param controllerMode {@link Control#Position} or {@link Control#Velocity}.
     * @param tolerance Error tolerance for {@link #atSetpoint()}.
     * @param resetPosition Position to which {@link #reset()} sets the motors, for example, the mechanism's home position.
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
     * @param measurement Supplies the mechanism's current measurement, for example, {@code leader::getPosition}.
     * @param tolerance Error tolerance for {@link #atSetpoint()}.
     * @param resetPosition Position to which {@link #reset()} sets the motors, for example, the mechanism's home position.
     * @param motorConfig Hardware configuration applied to every motor.
     * @param motors The motor(s) driven by this mechanism. The first is the leader.
     */
    public MechanismBase(PIDFFConfig gains, DoubleBinaryOperator feedback, DoubleConsumer resetController,
                          DoubleSupplier measurement, double tolerance, double resetPosition,
                          MotorConfig motorConfig, NAR_Motor... motors) {
        this(gains, null, feedback, resetController, measurement, tolerance, resetPosition, motorConfig, motors);
    }

    /**
     * Accepts either a {@code controllerMode} for a loop on the motor controller, or a
     * {@code feedback} for a loop on the roboRIO, with the other null. If both are null, the mechanism is open loop.
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
     * {@code gains}, for example, from Shuffleboard. Does nothing when the roboRIO runs the loop.
     */
    public void applyGains() {
        if (controllerMode == null) return;
        // kG calculated on the roboRIO must not also be applied by the motor controller
        final PIDFFConfig sent = kGOnController ? gains
            : new PIDFFConfig(gains.kP, gains.kI, gains.kD, gains.kS, gains.kV, gains.kA, () -> 0);
        for (NAR_Motor motor : commandedMotors) {
            if (motorRotationsPerArmRotation > 0) motor.configArmClosedLoop(sent, motorRotationsPerArmRotation);
            else motor.configClosedLoop(sent, controllerMode);
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
     * Leaves kG out of what the motor controller applies, for use when gravity depends on something
     * the motor controller cannot measure, for example, the angle of the arm under a wrist. Calculate kG in
     * {@link #calculateFeedforward(double)} instead; it is sent with each setpoint.
     */
    protected void useRoboRioGravity() {
        motorRotationsPerArmRotation = 0;
        kGOnController = false;
        applyGains();
    }

    /**
     * Creates an open-loop mechanism, with no PID or feedforward loop.
     * {@link #enable()} and {@link #setSetpoint(double)} are unavailable; use {@link #run(double)}
     * or {@link #runVolts(double)} directly.
     * The measurement defaults to the first motor's position so that Shuffleboard widgets still work.
     */
    public MechanismBase(MotorConfig motorConfig, NAR_Motor... motors) {
        this(new PIDFFConfig(), null, null, () -> motors[0].getPosition(), 0, 0, motorConfig, motors);
    }

    /** Reverses one motor relative to {@code motorConfig}, for example, when it is mounted facing the opposite way. */
    public void invertMotor(int motorIndex) {
        final NAR_Motor motor = motors[motorIndex];
        // A follower's direction is set relative to the leader rather than by its own inversion
        if (motorIndex == 0 || !motor.followOnController(motors[0], true)) {
            motor.setInverted(!motorConfig.inverted);
        }
    }

    /** Sets the stator current limit of every motor, for example, to push gently against a hard stop. */
    public void setStatorLimit(int amps) {
        for (NAR_Motor motor : motors) {
            motor.setStatorLimit(amps);
        }
    }

    /**
     * Calculates feedforward on the roboRIO, in volts. The default is 0.
     *
     * <p>When the roboRIO runs the loop, override this method to add, for example,
     * {@code gains.positionFF(pidOutput, atSetpoint())}. When the motor controller runs the loop
     * it already applies kS, kV and kG, so override this method only for anything further that it cannot do,
     * for example, a kG that changes with an elevator's height.
     *
     * @param pidOutput The output of {@code feedback} this cycle, or the error (setpoint minus
     *                  measurement) when the motor controller runs the loop. Use it to determine the sign of kS.
     */
    protected double calculateFeedforward(double pidOutput) {
        return 0;
    }

    /**
     * Runs every robot loop (50 times a second). While a setpoint is active, this sends the setpoint
     * to the motors, checks the safety timeout, and disables the loop at the setpoint if configured to. It always logs.
     */
    @Override
    public void periodic() {
        if (enabled) {
            if (controllerMode != null) {
                // The motor controller runs PID and feedforward; this adds only what it cannot calculate
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

        // Created on first use, since subclasses set the name after this constructor runs
        if (log == null) log = new MechanismLog(getName(), motors);
        log.update(this, measurement.getAsDouble());
    }

    /**
     * Returns the hardware settings applied to every motor: gearing, current limit, inversion and brake mode.
     */
    public MotorConfig getMotorConfig() {
        return motorConfig;
    }

    /**
     * Sets how long the mechanism may go without reaching its setpoint before it stops,
     * for example, because it is jammed. The timer restarts each time it reaches the setpoint.
     * @param timeSeconds The timeout in seconds; 5 by default
     */
    public void setSafetyThresh(double timeSeconds) {
        safetyThresh = timeSeconds;
    }

    /**
     * Called when the safety timeout expires. Logs a warning and stops the motors. Override this
     * method to change that behavior.
     */
    public void onSafetyTimeout(){
        Log.unusual(getName(), "Safety Timeout Reached");
        disable();
    }

    /**
     * Moves to {@code setpoint} and keeps the control loop running. The setpoint is limited to the range from
     * {@link #setInputRange(double, double)}. While the Shuffleboard DEBUG switch is on, the
     * Debug_Setpoint box is used instead.
     *
     * @param setpoint A position or speed, in the units of this mechanism
     */
    public void setSetpoint(double setpoint) {
        enable();
        final double requested = (debug != null && debug.getAsBoolean()) ? debugSetpoint.getAsDouble() : setpoint;
        setpointValue = MathUtil.clamp(requested, inputMin, inputMax);
        NAR_Shuffleboard.addData(getName(), "AtSetpoint", false, 1, 0);
    }

    /**
     * Returns a command that calls {@link #setSetpoint(double)} and finishes immediately, without
     * waiting to reach it. Follow it with {@code waitUntil(mechanism::atSetpoint)} to wait.
     */
    public Command setSetpointCommand(double setpoint) {
        return runOnce(() -> setSetpoint(setpoint));
    }

    /**
     * Sets whether the mechanism stops once {@link #atSetpoint()} is true. Defaults to true, which suits a
     * roller; set this to false for mechanisms that must hold position, such as an elevator.
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

    /** Returns the last setpoint, after limiting it to the input range. */
    public double getSetpoint() {
        return setpointValue;
    }

    /** Returns whether the measurement is within the tolerance of the setpoint. */
    public boolean atSetpoint() {
        return Math.abs(measurement.getAsDouble() - setpointValue) < tolerance;
    }

    /**
     * Starts the control loop without changing the setpoint. {@link #setSetpoint(double)} calls
     * this automatically. A roboRIO controller is reset first if the loop was not already running, so it
     * does not retain old state such as accumulated kI.
     */
    public void enable() {
        if (controllerMode == null) {
            requireNonNullParam(feedback, "feedback", "MechanismBase.enable");
            if (!enabled) resetController.accept(measurement.getAsDouble());
        }
        enabled = true;
        safetyTimer.restart();
        Log.debug(Log.Type.CONTROLLER, getName(), "Enabled PID");
    }

    /** Stops the control loop. If it was running, the motors also stop, so they do not retain their last output. */
    public void disable() {
        if (enabled) {
            for (NAR_Motor motor : commandedMotors) {
                motor.set(0);
            }
        }
        enabled = false;
        Log.debug(Log.Type.CONTROLLER, getName(), "Disabled PID");
    }

    /** Returns whether the control loop is running. */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Drives the motors at a fraction of full power, stopping the control loop.
     *
     * @param power From -1 (full reverse) to 1 (full forward)
     */
    public void run(double power) {
        disable();
        for (NAR_Motor motor : commandedMotors) {
            motor.set(power);
        }
    }

    /** Returns a command that calls {@link #run(double)} and finishes immediately; the motors keep running. */
    public Command runCommand(double power) {
        return runOnce(() -> run(power));
    }

    /**
     * Drives the motors at a voltage, stopping the control loop. Unlike {@link #run(double)}, the
     * motor receives the same voltage regardless of battery charge.
     *
     * @param volts From -12 to 12
     */
    public void runVolts(double volts) {
        disable();
        for (NAR_Motor motor : commandedMotors) {
            motor.setVolts(volts);
        }
    }

    /** Returns a command that calls {@link #runVolts(double)} and finishes immediately; the motors keep running. */
    public Command runVoltsCommand(double volts) {
        return runOnce(() -> runVolts(volts));
    }

    /** Stops the control loop and the motors. */
    public void stop() {
        disable();
        run(0);
    }

    /**
     * Tells the mechanism it is at its starting position, for example, after pushing it against a hard
     * stop, by setting every motor's encoder to the {@code resetPosition} given at construction.
     */
    public void reset() {
        reset(resetPosition);
    }

    /** Returns a command that calls {@link #reset()}, for example, for a re-zero button. */
    public Command resetCommand() {
        return runOnce(this::reset);
    }

    /** Sets every motor's encoder to {@code position}, stopping the control loop. */
    public void reset(double position) {
        disable();
        for (NAR_Motor motor : motors) {
            motor.resetPosition(position);
        }
    }

    /** Returns a command that calls {@link #reset(double)}. */
    public Command resetCommand(double position) {
        return runOnce(() -> reset(position));
    }

    /** Returns the position in this mechanism's units, measured by the leader's encoder. */
    public double getPosition() {
        return motors[0].getPosition();
    }

    /** Returns the velocity in this mechanism's units, measured by the leader's encoder. */
    public double getVelocity() {
        return motors[0].getVelocity();
    }

    /** Returns the voltage the leader is currently applying, from -12 to 12. */
    public double getVolts() {
        return motors[0].getAppliedOutput() * MAX_VOLTS;
    }

    /**
     * Mounts this mechanism on another, for example, an arm on an elevator, so that its 3D pose moves with the
     * parent. Subclasses also handle other effects of mounting, such as gravity.
     */
    public void setParent(MechanismBase parent) {
        if (this.parent != null) this.parent.mounted.remove(this);
        this.parent = parent;
        parent.mounted.add(this);
        // What is mounted on this mechanism may depend on what this mechanism is mounted on, such as for
        // gravity, so parents can be set in any order
        for (MechanismBase child : List.copyOf(mounted)) {
            child.setParent(this);
        }
    }

    /** Returns the mechanism on which this one is mounted, or null if it is mounted on the robot. */
    public MechanismBase getParent() {
        return parent;
    }

    /** Returns the mechanisms mounted directly on this one. */
    public List<MechanismBase> getMounted() {
        return Collections.unmodifiableList(mounted);
    }

    /**
     * Returns whether moving this mechanism tilts what is mounted on it, changing how gravity acts: true for
     * an arm, false for an elevator or turret unless mounted on something that tilts.
     */
    public boolean tiltsWhatsMountedOnIt() {
        return parent != null && parent.tiltsWhatsMountedOnIt();
    }

    /**
     * Returns the position of the moving part, relative to where it starts, for AdvantageScope's 3D
     * view. The robot's CAD should be exported with every mechanism at its starting position.
     */
    public final Pose3d getComponentPose() {
        final Pose3d own = getOwnComponentPose();
        return parent == null ? own : parent.getComponentPose().transformBy(new Transform3d(own.getTranslation(), own.getRotation()));
    }

    /** Returns the movement from the starting position, ignoring any parent. The default is none, as for rollers. */
    protected Pose3d getOwnComponentPose() {
        return Pose3d.kZero;
    }

    /**
     * Returns a SysId routine that drives the mechanism with rising and stepped voltages and records
     * the response, so that WPILib's SysId tool can calculate kS, kV, kA and kG. Each of the four tests
     * runs for up to 10 seconds, so allow room to move, and cancel the routine before a hard stop.
     *
     * @param rampRate Volts per second for the slowly rising test
     * @param stepVoltage Volts for the step test
     */
    public Command characterization(double rampRate, double stepVoltage) {
       NAR_SysIdCommand characterize = new NAR_SysIdCommand(rampRate, stepVoltage, (v) -> runVolts(v.in(Volts)), this, motors[0]);
       return characterize.runSysId();
    }

    /**
     * Adds the underlying WPILib controller's own Shuffleboard widget, since it already implements
     * {@link Sendable}), for example, {@code addControllerWidget(pid)}.
     */
    protected void addControllerWidget(Sendable controller, int x, int y, int width, int height) {
        NAR_Shuffleboard.addSendable(getName(), "PID_Controller", controller, x, y, width, height).withWidget(BuiltInWidgets.kPIDController);
    }

    /**
     * Adds a Shuffleboard tab for tuning: the setpoint and measurement with graphs, boxes for
     * the feedforward gains, and buttons to run the mechanism. The DEBUG switch enables the buttons
     * and the Debug_Setpoint box; while it is on, the box overrides every setpoint.
     */
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

    /** Adds a box and button to run the mechanism at a voltage, for testing direction and wiring. */
    private void runVoltsWidgets(BooleanSupplier debug, int x, int y) {
        final DoubleSupplier debugVoltage = NAR_Shuffleboard.debug(getName(), "Debug Volts", 0, x + 6, y + 3);
        NAR_Shuffleboard.addCommand(getName(), "Run Volts", either(startEnd(() -> runVolts(debugVoltage.getAsDouble()), this::stop), print("DEBUG NOT ON"), debug), x + 7, y + 3);
        NAR_Shuffleboard.addData(getName(), "Running", () -> debug.getAsBoolean() && getVolts() > 0, x + 5, y + 3);
        NAR_Shuffleboard.addData(getName(), "Voltage", this::getVolts, x + 4, y + 3);
    }

    /**
     * Adds boxes for the feedforward gains, which the gains then read from, and a SysId button. When the
     * motor controller runs the loop, also adds a button that sends new gains to it.
     */
    private void FFWidgets(int x, int y) {
        gains.setkS(NAR_Shuffleboard.debug(getName(), "kS", gains.getkS(), x, y + 2));
        gains.setkV(NAR_Shuffleboard.debug(getName(), "kV", gains.getkV(), x + 1, y + 2));
        gains.setkA(NAR_Shuffleboard.debug(getName(), "kA", gains.getkA(), x + 1, y + 3));
        gains.setkG(NAR_Shuffleboard.debug(getName(), "kG", gains.getkG(), x, y + 3));
        NAR_Shuffleboard.addCommand(getName(), "Characterize", characterization(1, 0.5), x - 1, y + 3).withSize(1, 1);
        if (controllerMode != null) {
            NAR_Shuffleboard.addCommand(getName(), "Apply Gains", runOnce(this::applyGains).ignoringDisable(true), x + 2, y + 2);
        }
    }
}
