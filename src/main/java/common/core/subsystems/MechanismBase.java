package common.core.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
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
 * Base class for a PID-controlled mechanism.
 *
 * <p>This class does not define its own controller type. Instead it drives a plain
 * WPILib controller (e.g. {@link edu.wpi.first.math.controller.PIDController},
 * {@link edu.wpi.first.math.controller.ProfiledPIDController},
 * {@link edu.wpi.first.math.controller.BangBangController}) directly: those classes are
 * pure {@code (measurement, setpoint) -> output} calculators and know nothing about
 * motors, so {@code MechanismBase} is the one responsible for reading the measurement,
 * calling the controller, adding feedforward, and applying the result to its motors.
 *
 * <p>Feedforward (kS/kV/kA/kG) is not part of the injected controller either, since
 * WPILib's feedforward classes don't support the dynamic kG use this team needs (e.g. a
 * pivot's gravity gain varying with angle). Subclasses instead override
 * {@link #calculateFeedforward(double)} to combine {@link PIDFFConfig} with whatever
 * shape of feedforward their mechanism needs.
 */
public abstract class MechanismBase extends SubsystemBase {

    protected final NAR_Motor[] motors;
    protected final PIDFFConfig gains;

    /** Computes controller output from (measurement, setpoint), e.g. {@code pid::calculate}. */
    private final DoubleBinaryOperator feedback;
    /** Resets the controller's internal state, given the current measurement. */
    private final DoubleConsumer resetController;
    private final DoubleSupplier measurement;
    private final double tolerance;

    private double setpointValue;
    private boolean enabled = false;

    private double safetyThresh = 5;
    private final Timer safetyTimer = new Timer();
    private MotorConfig motorConfig;

    protected BooleanSupplier debug;
    protected DoubleSupplier debugSetpoint;

    protected static List<MechanismBase> instances = new ArrayList<>();

    /**
     * @param gains Feedforward gains; also handed to {@link #calculateFeedforward(double)}.
     * @param feedback The WPILib controller's {@code calculate(measurement, setpoint)} method reference.
     * @param resetController Resets the controller's internal state (integral, previous error, profile) given the
     *                        current measurement; called when the mechanism goes from disabled to enabled. Use
     *                        {@code m -> pid.reset()} for a {@code PIDController}, {@code profiled::reset} for a
     *                        {@code ProfiledPIDController}, or {@code m -> {}} for a {@code BangBangController}.
     * @param measurement Supplies the mechanism's current measurement, e.g. {@code leader::getPosition}.
     * @param tolerance Error tolerance for {@link #atSetpoint()}.
     * @param motorConfig Hardware configuration applied to every motor.
     * @param motors The motor(s) driven by this mechanism. Output is applied to all of them.
     */
    public MechanismBase(PIDFFConfig gains, DoubleBinaryOperator feedback, DoubleConsumer resetController,
                          DoubleSupplier measurement, double tolerance, MotorConfig motorConfig, NAR_Motor... motors) {
        requireNonNullParam(motors, "motors", "MechanismBase");
        requireNonNullParam(gains, "gains", "MechanismBase");
        if (motors.length == 0) {
            throw new IllegalArgumentException("MechanismBase requires at least one motor");
        }
        for (int i = 0; i < motors.length; i++) {
            requireNonNullParam(motors[i], "motors[" + i + "]", "MechanismBase");
        }

        this.gains = gains;
        this.feedback = feedback;
        this.resetController = resetController;
        this.measurement = measurement;
        this.tolerance = tolerance;
        this.motorConfig = motorConfig;
        this.motors = motors;

        for (NAR_Motor motor : motors) {
            motor.configMotor(motorConfig);
        }
    }

    /**
     * Creates an open-loop-only mechanism, with no PID/feedforward loop.
     * {@link #enable()}/{@link #setSetpoint(double)} are unavailable; use {@link #run(double)}
     * or {@link #runVolts(double)} directly.
     * The measurement defaults to the first motor's position so Shuffleboard widgets still work.
     */
    public MechanismBase(MotorConfig motorConfig, NAR_Motor... motors) {
        this(new PIDFFConfig(), null, null, () -> motors[0].getPosition(), 0, motorConfig, motors);
    }

    public void invertMotor(int motorIndex) {
        motors[motorIndex].setInverted(!motorConfig.inverted);
    }

    /**
     * Combines the controller's PID output with this mechanism's feedforward.
     * Default is no feedforward; override to add e.g. {@code gains.positionFF(pidOutput, atSetpoint())}
     * or {@code gains.velocityFF(getSetpoint(), pidOutput, atSetpoint())}.
     *
     * @param pidOutput The output of {@code feedback} this cycle.
     */
    protected double calculateFeedforward(double pidOutput) {
        return 0;
    }

    /**
     * This is an extendable implementation of the singleton pattern<br><br>
     *
     * When getting a mechanism instance whose class name is [CLASS_NAME], use the following code:<br>
     * <strong>[CLASS_NAME] mechanism = [CLASS_NAME].getInstance([CLASS_NAME].class);</strong><br>
     *
     * @param type [MECHANISM_CLASS_NAME].class
     * @return a singleton instance of the mechanism
     */
    public static <T extends MechanismBase> T getInstance(Class<T> type) {
        for (MechanismBase instance : instances) {
            if (type.isInstance(instance)) {
                return type.cast(instance);
            }
        }

        MechanismBase instance;
        try {
            instance = type.getDeclaredConstructor().newInstance();
            instances.add(instance);
        } catch (ReflectiveOperationException e) {
            throw new RuntimeException("Failed to instantiate " + type.getName(), e);
        }

        return type.cast(instance);
    }

    @Override
    public void periodic() {
        if (enabled) {
            final double pidOutput = feedback.applyAsDouble(measurement.getAsDouble(), setpointValue);
            final double output = MathUtil.clamp(pidOutput + calculateFeedforward(pidOutput), -12, 12);
            for (NAR_Motor motor : motors) {
                motor.setVolts(output);
            }

            if (safetyTimer.hasElapsed(safetyThresh)) onSafetyTimeout();
            if (atSetpoint()) {
                safetyTimer.restart();
                NAR_Shuffleboard.addData(getName(), "AtSetpoint", true, 1, 0);
                disable();
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
     * Disables the PID control.
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
        setpointValue = (debug != null && debug.getAsBoolean()) ? debugSetpoint.getAsDouble() : setpoint;
        NAR_Shuffleboard.addData(getName(), "AtSetpoint", false, 1, 0);
    }

    public Command setSetpointCommand(double setpoint) {
        return runOnce(() -> setSetpoint(setpoint));
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
        requireNonNullParam(feedback, "feedback", "MechanismBase.enable");
        if (!enabled) resetController.accept(measurement.getAsDouble());
        enabled = true;
        safetyTimer.restart();
        Log.debug(Log.Type.CONTROLLER, getName(), "Enabled PID");
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
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
     * Sets power to motors.
     *
     * @param power The power to set the motors to between -1 and 1.
     */
    public void run(double power) {
        for (NAR_Motor motor : motors) {
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
     * Sets voltage to motors.
     *
     * @param volts The voltage to set the motors to.
     */
    public void runVolts(double volts) {
        for (NAR_Motor motor : motors) {
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
     * Resets every motor's position to {@code position}.
     */
    public void reset(double position) {
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
        return motors[0].getAppliedOutput() * 12;
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
    }
}
