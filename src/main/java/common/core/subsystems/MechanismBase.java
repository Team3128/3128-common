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
 * What every mechanism has in common: one or more motors, driven to a setpoint, with logging,
 * Shuffleboard tuning widgets and a safety timeout.
 *
 * <p>You'll usually use one of its subclasses instead of this class itself:
 * {@link LinearMechanism} for elevators and sliders, {@link PivotMechanism} for arms, wrists,
 * hoods and turrets, and {@link RollerMechanism} for intakes and flywheels.
 *
 * <h2>Using a mechanism</h2>
 * <ul>
 * <li>{@link #setSetpoint(double)} moves it to a position or speed, in its own units (set by
 *     {@code MotorConfig}), and keeps it there.</li>
 * <li>{@link #run(double)} and {@link #runVolts(double)} drive it directly, without a setpoint.</li>
 * <li>{@link #stop()} stops it; {@link #reset()} tells it it's at its starting position, e.g.
 *     when it's against a hard stop.</li>
 * </ul>
 *
 * <h2>Where the control loop runs</h2>
 * A control loop measures the mechanism, compares it with the setpoint, and works out the volts
 * to send (see {@link PIDFFConfig} for the gains). It can run in one of two places:
 * <ul>
 * <li><b>On the motor controller</b>, the usual choice. The TalonFX or Spark runs PID and
 *     feedforward 1000 times a second on its own encoder, 50 times faster than robot code runs.
 *     Gains are given in the mechanism's units and converted for the motor controller.
 *     <pre>{@code super(gains, Control.Position, TOLERANCE, HOME, MOTOR_CONFIG, motor);}</pre></li>
 * <li><b>On the roboRIO</b>, for a WPILib controller like a {@code BangBangController}, or when the
 *     measurement doesn't come from the motor's encoder, e.g. a gyro or camera. The controller
 *     works out volts from {@code (measurement, setpoint)}:
 *     <pre>{@code super(gains, pid::calculate, m -> pid.reset(), gyro::getAngle, TOLERANCE, HOME, MOTOR_CONFIG, motor);}</pre></li>
 * </ul>
 *
 * <p>Anything the motor controller can't work out itself, like gravity on a wrist that depends on
 * the arm it's on, goes in {@link #calculateFeedforward(double)}, which runs on the roboRIO and is
 * sent along with each setpoint.
 *
 * <h2>Several motors</h2>
 * When several motors drive one mechanism, the first is the leader and the rest follow it on the
 * motor controller where possible. Only the leader runs the control loop.
 *
 * <h2>Logging and 3D</h2>
 * Every loop, the setpoint, measurement, volts, and each motor's current and temperature are
 * logged under {@code Mechanisms/<name>} in NetworkTables, for AdvantageScope.
 * {@link #getComponentPose()} gives where its moving part is for AdvantageScope's 3D view, and
 * {@link #setParent(MechanismBase)} mounts it on another mechanism, e.g. a wrist on an arm.
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
    /** False when kG is worked out on the roboRIO instead, in {@link #calculateFeedforward(double)}. */
    private boolean kGOnController = true;
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
    private MechanismLog log;
    /** What it's mounted on, for the 3D view; null if the robot itself. */
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
        // kG worked out on the roboRIO mustn't be applied by the motor controller too
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
     * Leaves kG out of what the motor controller applies, for when gravity depends on something
     * it can't measure, e.g. a wrist's on the arm it's mounted on. Work kG out in
     * {@link #calculateFeedforward(double)} instead; it's sent with each setpoint.
     */
    protected void useRoboRioGravity() {
        motorRotationsPerArmRotation = 0;
        kGOnController = false;
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

    /**
     * Runs every robot loop (50 times a second). While a setpoint is active, it sends the setpoint
     * to the motors, checks the safety timeout, and stops if it should once there. It always logs.
     */
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

            // Gives up if it hasn't been at the setpoint for too long, e.g. because it's jammed
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
     * The hardware settings every motor got: gearing, current limit, inversion and brake mode.
     */
    public MotorConfig getMotorConfig() {
        return motorConfig;
    }

    /**
     * Sets how long it can go without being at its setpoint before it gives up and stops,
     * e.g. because it's jammed. The clock restarts every time it reaches the setpoint.
     * @param timeSeconds How long, in seconds; 5 by default
     */
    public void setSafetyThresh(double timeSeconds) {
        safetyThresh = timeSeconds;
    }

    /**
     * Called when the safety timeout runs out. Logs a warning and stops the motors. Override it
     * to do something else.
     */
    public void onSafetyTimeout(){
        Log.unusual(getName(), "Safety Timeout Reached");
        disable();
    }

    /**
     * Moves to {@code setpoint} and keeps the control loop running. It's limited to the range from
     * {@link #setInputRange(double, double)}. While the Shuffleboard DEBUG switch is on, the
     * Debug_Setpoint box is used instead.
     *
     * @param setpoint A position or speed, in this mechanism's units
     */
    public void setSetpoint(double setpoint) {
        enable();
        final double requested = (debug != null && debug.getAsBoolean()) ? debugSetpoint.getAsDouble() : setpoint;
        setpointValue = MathUtil.clamp(requested, inputMin, inputMax);
        NAR_Shuffleboard.addData(getName(), "AtSetpoint", false, 1, 0);
    }

    /**
     * A command that calls {@link #setSetpoint(double)} and finishes right away, without waiting
     * to get there. Follow it with {@code waitUntil(mechanism::atSetpoint)} to wait.
     */
    public Command setSetpointCommand(double setpoint) {
        return runOnce(() -> setSetpoint(setpoint));
    }

    /**
     * Sets whether it stops once {@link #atSetpoint()} is true. Defaults to true, which suits a
     * roller; set it to false for mechanisms that must keep holding, e.g. an elevator.
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

    /** The last setpoint, after limiting it to the input range. */
    public double getSetpoint() {
        return setpointValue;
    }

    /** Whether the measurement is within the tolerance of the setpoint. */
    public boolean atSetpoint() {
        return Math.abs(measurement.getAsDouble() - setpointValue) < tolerance;
    }

    /**
     * Starts the control loop, without changing the setpoint. {@link #setSetpoint(double)} calls
     * this for you. A roboRIO controller is reset first if the loop wasn't already running, so it
     * doesn't start with old leftovers like a built-up kI.
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

    /** Stops the control loop. If it was running, the motors stop too, so they don't keep their last output. */
    public void disable() {
        if (enabled) {
            for (NAR_Motor motor : commandedMotors) {
                motor.set(0);
            }
        }
        enabled = false;
        Log.debug(Log.Type.CONTROLLER, getName(), "Disabled PID");
    }

    /** Whether the control loop is running. */
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

    /** A command that calls {@link #run(double)} and finishes right away; the motors keep running. */
    public Command runCommand(double power) {
        return runOnce(() -> run(power));
    }

    /**
     * Drives the motors at a voltage, stopping the control loop. Unlike {@link #run(double)}, the
     * motor gets the same push however charged the battery is.
     *
     * @param volts From -12 to 12
     */
    public void runVolts(double volts) {
        disable();
        for (NAR_Motor motor : commandedMotors) {
            motor.setVolts(volts);
        }
    }

    /** A command that calls {@link #runVolts(double)} and finishes right away; the motors keep running. */
    public Command runVoltsCommand(double volts) {
        return runOnce(() -> runVolts(volts));
    }

    /** Stops the control loop and the motors. */
    public void stop() {
        disable();
        run(0);
    }

    /**
     * Tells the mechanism it's at its starting position, e.g. after pushing it against a hard
     * stop, by setting every motor's encoder to the {@code resetPosition} given at construction.
     */
    public void reset() {
        reset(resetPosition);
    }

    /** A command that calls {@link #reset()}, e.g. for a re-zero button. */
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

    /** A command that calls {@link #reset(double)}. */
    public Command resetCommand(double position) {
        return runOnce(() -> reset(position));
    }

    /** Where it is, in its own units, measured by the leader's encoder. */
    public double getPosition() {
        return motors[0].getPosition();
    }

    /** How fast it's moving, in its own units, measured by the leader's encoder. */
    public double getVelocity() {
        return motors[0].getVelocity();
    }

    /** The volts the leader is putting out right now, from -12 to 12. */
    public double getVolts() {
        return motors[0].getAppliedOutput() * MAX_VOLTS;
    }

    /**
     * Mounts this on another mechanism, e.g. an arm on an elevator, so its 3D pose moves with the
     * parent's. Subclasses also handle anything else being mounted changes, e.g. gravity.
     */
    public void setParent(MechanismBase parent) {
        if (this.parent != null) this.parent.mounted.remove(this);
        this.parent = parent;
        parent.mounted.add(this);
        // What's mounted on this may depend on what this is mounted on, e.g. for gravity, so
        // parents can be set in any order
        for (MechanismBase child : List.copyOf(mounted)) {
            child.setParent(this);
        }
    }

    /** What it's mounted on; null if the robot itself. */
    public MechanismBase getParent() {
        return parent;
    }

    /** What's mounted directly on it. */
    public List<MechanismBase> getMounted() {
        return Collections.unmodifiableList(mounted);
    }

    /**
     * Whether moving this tilts what's mounted on it, changing how gravity acts on it: true for
     * an arm, false for an elevator or turret, unless they're mounted on something that does.
     */
    public boolean tiltsWhatsMountedOnIt() {
        return parent != null && parent.tiltsWhatsMountedOnIt();
    }

    /**
     * Where the mechanism's moving part is, relative to where it starts, for AdvantageScope's 3D
     * view. Export the robot's CAD with every mechanism at its starting position.
     */
    public final Pose3d getComponentPose() {
        final Pose3d own = getOwnComponentPose();
        // Move with the parent, as if attached to it
        return parent == null ? own : parent.getComponentPose().transformBy(new Transform3d(own.getTranslation(), own.getRotation()));
    }

    /** How far it has moved from where it starts on its own, ignoring any parent. Default is none, e.g. for rollers. */
    protected Pose3d getOwnComponentPose() {
        return Pose3d.kZero;
    }

    /**
     * A SysId routine that drives the mechanism with rising and stepped voltages and records how
     * it responds, so WPILib's SysId tool can work out its kS, kV, kA and kG. Each of its four tests
     * runs for up to 10 seconds, so give it room to move, and cancel it before it hits a hard stop.
     *
     * @param rampRate Volts per second for the slowly rising test
     * @param stepVoltage Volts for the sudden-step test
     */
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

    /**
     * Adds a Shuffleboard tab for tuning: the setpoint and measurement with graphs, boxes to type
     * the feedforward gains, and buttons to run it. Turn on the DEBUG switch to use the buttons
     * and the Debug_Setpoint box; while it's on, the box overrides every setpoint.
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

    /** A box and button to run it at a voltage, for testing direction and wiring. */
    private void runVoltsWidgets(BooleanSupplier debug, int x, int y) {
        final DoubleSupplier debugVoltage = NAR_Shuffleboard.debug(getName(), "Debug Volts", 0, x + 6, y + 3);
        NAR_Shuffleboard.addCommand(getName(), "Run Volts", either(startEnd(() -> runVolts(debugVoltage.getAsDouble()), this::stop), print("DEBUG NOT ON"), debug), x + 7, y + 3);
        NAR_Shuffleboard.addData(getName(), "Running", () -> debug.getAsBoolean() && getVolts() > 0, x + 5, y + 3);
        NAR_Shuffleboard.addData(getName(), "Voltage", this::getVolts, x + 4, y + 3);
    }

    /** Boxes for the feedforward gains, which the gains then read from, and a SysId button. */
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
