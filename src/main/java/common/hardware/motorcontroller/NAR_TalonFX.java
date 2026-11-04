package common.hardware.motorcontroller;

import static common.hardware.motorcontroller.MotorControllerConstants.HIGH_PRIORITY_FREQ;
import static common.hardware.motorcontroller.MotorControllerConstants.LOW_PRIORITY_FREQ;
import static common.hardware.motorcontroller.MotorControllerConstants.MEDIUM_PRIORITY_FREQ;
import static common.hardware.motorcontroller.MotorControllerConstants.NEO_STATOR_CurrentLimit;
import static common.hardware.motorcontroller.MotorControllerConstants.NEO_SUPPLY_CurrentLimit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import common.core.controllers.PIDFFConfig;
import common.utility.Log;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.RobotController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

/**
 * A CTRE TalonFX motor controller, which drives Kraken X60, Kraken X44 and Falcon 500 motors,
 * using Phoenix 6.
 *
 * <p>All settings are kept in one {@code TalonFXConfiguration}. Changing a setting updates that object and
 * sends the changed section to the Talon, retrying if the Talon does not confirm it. While a
 * {@link common.core.misc.NAR_Robot} starts up, settings are instead stored and sent together; see
 * {@link #deferSetup()}.
 *
 * <p>The Talon's own control loops measure in rotations and rotations per second, output volts,
 * and work in seconds. {@link NAR_Motor} converts gains and setpoints to and from the configured units.
 *
 * <p>Readings use Phoenix 6 status signals, which hold the latest values the Talon has reported.
 * {@code refresh()} reads the newest value without waiting on the CAN bus. Because
 * {@code optimizeBusUtilization()} turns off every signal whose rate has not been set, each
 * {@code set...StatusFrames} method sets a rate for every signal that is read: the important ones
 * for that kind of motor every 20 ms, and the rest, used mostly for logs, less often.
 */
public class NAR_TalonFX extends NAR_Motor {

    // Settings can fail on the background thread too
    private static final AtomicInteger numFailedConfigs = new AtomicInteger();

	/** The number of attempts to send a setting before failing. */
	public static int maximumRetries = 5;

    /** Sends settings for every Talon one at a time, in order, off the robot thread. */
    private static final ExecutorService BACKGROUND_CONFIGS = Executors.newSingleThreadExecutor(runnable -> {
        final Thread thread = new Thread(runnable, "TalonFX configs");
        thread.setDaemon(true);
        return thread;
    });

    /** Returns the number of settings, across every Talon, that failed after all retries. */
	public static int getNumFailedConfigs() {
		return numFailedConfigs.get();
	}

    /** While true, settings are only stored, and {@link #finishSetupAll()} sends them. */
    // Volatile because this class also runs a background thread
    private static volatile boolean deferringSetup = false;
    /** Talons with settings waiting for {@link #finishSetupAll()}, in the order they were created. */
    private static final List<NAR_TalonFX> deferredTalons = new ArrayList<>();

    /**
     * Stores every Talon's settings instead of sending them, until {@link #finishSetupAll()}. Each
     * setting is a blocking call that waits for the Talon, so sending them together instead of one
     * at a time shortens startup. {@link common.core.misc.NAR_Robot} calls both.
     */
    public static void deferSetup() {
        deferringSetup = true;
    }

    /**
     * Sends every setting stored since {@link #deferSetup()}: one configuration per Talon, one call per
     * status signal rate for every Talon together, then encoder positions, which need the Talon's
     * gearing to be set first. Afterwards, settings are sent as they change.
     */
    public static void finishSetupAll() {
        if (!deferringSetup) return;
        deferringSetup = false;

        for (NAR_TalonFX talon : deferredTalons) {
            talon.configTalonFX(() -> talon.motor.getConfigurator().apply(talon.configs));
        }

        sendFrequencies(deferredTalons);

        for (NAR_TalonFX talon : deferredTalons) {
            if (talon.pendingRotorRotations == null) continue;
            final double rotations = talon.pendingRotorRotations;
            talon.configTalonFX(() -> talon.motor.setPosition(rotations / talon.mechanismRatio));
        }

        for (NAR_TalonFX talon : deferredTalons) {
            talon.pendingFrequencies.clear();
            talon.pendingRotorRotations = null;
            talon.awaitingSetup = false;
        }
        deferredTalons.clear();
    }

    private final TalonFX motor;

    private final CANBus canBus;

    /** Every setting for this Talon; sections are sent as they change. */
    TalonFXConfiguration configs = new TalonFXConfiguration();
    /** Rotor rotations per rotation reported by the Talon, from SensorToMechanismRatio. */
    private double mechanismRatio = 1;

    private final StatusSignal<Double> appliedOutput;
    private final StatusSignal<Current> stallCurrent;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Integer> faults;

    /** Whether this Talon is in {@link #deferredTalons}. */
    private boolean awaitingSetup = false;
    /** Status signal rates in hertz, waiting for {@link #finishSetupAll()}. */
    private final Map<BaseStatusSignal, Double> pendingFrequencies = new LinkedHashMap<>();
    /** An encoder position in rotor rotations, waiting for {@link #finishSetupAll()}, or null. */
    private Double pendingRotorRotations = null;

    /**
     * @param deviceNumber The Talon's CAN ID
     * @param canbusStr The CAN bus name: {@code "rio"} for the roboRIO's bus, or a CANivore's name
     * @param pidConfig Initial PID gains, in the Talon's own units
     */
    public NAR_TalonFX(int deviceNumber, String canbusStr, PIDFFConfig pidConfig) {
        super(deviceNumber);
        canBus = new CANBus(canbusStr);
        motor = new TalonFX(deviceNumber, canBus);

        appliedOutput = motor.getDutyCycle();
        stallCurrent = motor.getStatorCurrent();
        position = motor.getPosition();
        velocity = motor.getVelocity();
        temperature = motor.getDeviceTemp();
        faults = motor.getFaultField();

        // Safe settings until a mechanism's MotorConfig replaces them
        enableVoltageCompensationNoApply(12);
        setCurrentLimitNoApply(NEO_STATOR_CurrentLimit, NEO_SUPPLY_CurrentLimit);
        configPIDNoApply(pidConfig);
        apply();
    }

    /**
     * @param deviceNumber The Talon's CAN ID
     * @param canBus The CAN bus name: {@code "rio"} for the roboRIO's bus, or a CANivore's name
     */
    public NAR_TalonFX(int deviceNumber, String canBus) {
        this(deviceNumber, canBus, new PIDFFConfig());
    }

    /**
     * Creates a Talon on the CANivore named {@code "drivetrain"}. Use the other constructors for a
     * different CAN bus.
     * @param deviceNumber The Talon's CAN ID
     */
    public NAR_TalonFX(int deviceNumber) {
        this(deviceNumber, "drivetrain");
    }

    /**
	 * Sends a setting, making up to {@link #maximumRetries} attempts until the Talon confirms it.
	 *
	 * @param config Sends the setting and returns the result
	 */
	private void configTalonFX(Supplier<StatusCode> config)
	{
		sendWithRetries(config, "Failed to configure Talon FX " + motor.getDeviceID());
	}

	private static void sendWithRetries(Supplier<StatusCode> config, String failureMessage)
	{
		for (int i = 0; i < maximumRetries; i++)
		{
			if (config.get() == StatusCode.OK)
			{
				return;
			}
		}
		numFailedConfigs.incrementAndGet();
		Log.recoverable("Motors", failureMessage);
	}

    /**
     * Sends part of {@link #configs}, or during {@link #deferSetup()}, leaves it for
     * {@link #finishSetupAll()}, which sends the whole configuration at once.
     */
    private void sendConfig(Supplier<StatusCode> config) {
        if (deferSetupHere()) return;
        configTalonFX(config);
    }

    /** Returns true, and queues this Talon for {@link #finishSetupAll()}, if setup is deferred. */
    private boolean deferSetupHere() {
        if (!deferringSetup) return false;
        if (!awaitingSetup) {
            awaitingSetup = true;
            deferredTalons.add(this);
        }
        return true;
    }

    @Override
    public void apply() {
        sendConfig(() -> motor.getConfigurator().apply(configs));
    }

    @Override
    public void configPID(PIDFFConfig config) {
        configPIDNoApply(config);
        sendConfig(()-> motor.getConfigurator().apply(configs.Slot0));
    }

    @Override
    public void configPIDNoApply(PIDFFConfig config) {
        configs.Slot0.kP = config.kP;
        configs.Slot0.kI = config.kI;
        configs.Slot0.kD = config.kD;
    }

    @Override
    protected void configNativeClosedLoop(Control mode, double kP, double kI, double kD,
                                          double kS, double kV, double kG, double motorRotationsPerArmRotation) {
        configs.Slot0.kP = kP;
        configs.Slot0.kI = kI;
        configs.Slot0.kD = kD;
        configs.Slot0.kS = kS;
        configs.Slot0.kV = kV;
        configs.Slot0.kG = kG;
        configs.Slot0.GravityType = motorRotationsPerArmRotation > 0 ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
        // A position request has no velocity from which to determine the sign of kS, so the error is used
        configs.Slot0.StaticFeedforwardSign = mode == Control.Position
            ? StaticFeedforwardSignValue.UseClosedLoopSign
            : StaticFeedforwardSignValue.UseVelocitySign;
        sendConfig(()-> motor.getConfigurator().apply(configs.Slot0));
    }

    /**
     * Arm_Cosine requires the Talon's position in arm rotations, so the Talon divides by this ratio.
     * The rest of this class still uses rotor rotations.
     */
    @Override
    protected void setMechanismRatio(double motorRotationsPerMechanismRotation) {
        mechanismRatio = motorRotationsPerMechanismRotation;
        configs.Feedback.SensorToMechanismRatio = mechanismRatio;
        sendConfig(()-> motor.getConfigurator().apply(configs.Feedback));
    }

    @Override
    protected double getNativePositionRotations() {
        return mechanismRatio;
    }

    @Override
    protected double getNativeVelocityRPM() {
        return 60 * mechanismRatio;
    }

    @Override
    protected double getNativeOutputVolts() {
        return 1;
    }

    @Override
    protected double getNativeTimeSeconds() {
        return 1;
    }

    @Override
    public boolean followOnController(NAR_Motor leader, boolean opposeLeader) {
        if (!(leader instanceof NAR_TalonFX talon)) return false;
        // Both motors must be on the same CAN bus. The follower's direction comes from the
        // leader's Inverted config and the alignment; its own Inverted config is not used.
        motor.setControl(new Follower(talon.motor.getDeviceID(),
            opposeLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        return true;
    }

    @Override
    public void setInverted(boolean inverted) {
        setInvertedNoApply(inverted);
        sendConfig(()-> motor.getConfigurator().apply(configs.MotorOutput));
    }

    @Override
    public void setInvertedNoApply(boolean inverted) {
        configs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    }

    @Override
    protected void setPercentOutput(double speed) {
        motor.set(speed);
    }

    /**
     * Drives the motor at a voltage using field-oriented control (FOC), a commutation method that
     * produces more torque and power from Krakens and Falcons. Requires a Phoenix Pro license.
     * @param voltage From -12 to 12
     */
    public void setVoltageFOC(double voltage) {
        var voltageSetpoint = new VoltageOut(voltage)
            .withEnableFOC(true);
        motor.setControl(voltageSetpoint);
    }

    @Override
    protected void setVelocity(double rpm, double feedForward) {
        var velocitySetpoint = new VelocityVoltage(rpm / 60 / mechanismRatio);
        velocitySetpoint.FeedForward = feedForward;
        motor.setControl(velocitySetpoint);
    }

    /**
     * Holds a speed using FOC; see {@link #setVoltageFOC(double)}.
     * @param rpm Speed of the encoder in rotations per minute
     * @param feedforward Additional volts
     */
    public void setVelocityFOC(double rpm, double feedforward) {
        var velocitySetpoint = new VelocityVoltage(rpm / 60 / mechanismRatio)
            .withEnableFOC(true)
            .withFeedForward(feedforward);
        motor.setControl(velocitySetpoint);
    }

    /**
     * Holds a speed using FOC; see {@link #setVoltageFOC(double)}.
     * @param setpoint The speed in the configured units, as in {@link #set(double, Control)}
     * @param feedforward Additional volts
     */
    public void setVelocityFOC2(double setpoint, double feedforward) {
        setVelocityFOC(setpoint / unitConversionFactor * timeConversionFactor, feedforward);
    }

    @Override
    protected void setPosition(double rotations, double feedForward) {
        var positionSetpoint = new PositionVoltage(rotations / mechanismRatio);
        positionSetpoint.FeedForward = feedForward;
        motor.setControl(positionSetpoint);
    }

    /**
     * Moves to a position using FOC; see {@link #setVoltageFOC(double)}.
     * @param rotations Position of the encoder in rotations
     * @param feedforward Additional volts
     */
    public void setPositionFOC(double rotations, double feedforward) {
        var positionSetpoint = new PositionVoltage(rotations / mechanismRatio)
            .withEnableFOC(true)
            .withFeedForward(feedforward);
        motor.setControl(positionSetpoint);
    }

    @Override
    protected void resetRawPosition(double rotations) {
        // The Talon converts with its own SensorToMechanismRatio, which isn't sent until setup finishes
        if (deferSetupHere()) {
            pendingRotorRotations = rotations;
            return;
        }
        motor.setPosition(rotations / mechanismRatio);
    }

    @Override
    protected void simulationStep(double dtSeconds) {
        // Follows CTRE's TalonFXSimState pattern; the Talon applies inversion itself
        final TalonFXSimState simState = motor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        simModel.update(simState.getMotorVoltage(), dtSeconds);
        simState.setRawRotorPosition(simModel.getRotorRotations());
        simState.setRotorVelocity(simModel.getRotorRPS());
    }

    @Override
    public double getAppliedOutput() {
        return appliedOutput.refresh().getValue();
    }

    @Override
    public double getStallCurrent() {
        return stallCurrent.refresh().getValue().in(Units.Amp);
    }

    @Override
	public double getTorque() {
		// The Talon reports the torque constant of its motor, so this works for X60s, X44s and Falcons
		return motor.getTorqueCurrent().getValueAsDouble() * motor.getMotorKT().getValueAsDouble();
	}

    @Override
    protected double getRawPosition() {
        return position.refresh().getValue().in(Units.Revolution) * mechanismRatio;
    }

    @Override
    protected double getRawVelocity() {
        return velocity.refresh().getValue().in(Units.RotationsPerSecond) * 60.0 * mechanismRatio;
    }

    @Override
    public double getTemperature() {
        return temperature.refresh().getValue().in(Units.Celsius);
    }

    @Override
    protected void setBrakeMode() {
        setBrakeModeNoApply();
        sendConfig(()-> motor.getConfigurator().apply(configs.MotorOutput));
    }

    @Override
    protected void setBrakeModeNoApply() {
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    @Override
    protected void setCoastMode() {
        setCoastModeNoApply();
        sendConfig(()-> motor.getConfigurator().apply(configs.MotorOutput));
    }

    @Override
    protected void setCoastModeNoApply() {
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    @Override
    public void enableVoltageCompensation(double volts) {
        enableVoltageCompensationNoApply(volts);
        sendConfig(()-> motor.getConfigurator().apply(configs.Voltage));
    }

    @Override
    public void enableVoltageCompensationNoApply(double volts) {
        configs.Voltage.PeakForwardVoltage = volts;
        configs.Voltage.PeakReverseVoltage = -volts;
    }

    @Override
    public void setStatorLimit(int limit) {
        setStatorLimitNoApply(limit);
        sendConfig(()-> motor.getConfigurator().apply(configs.CurrentLimits));
    }

    /**
     * Like {@link #setStatorLimit}, but sends the limit from a background thread so the robot loop
     * doesn't wait up to 100 ms per attempt for the Talon to confirm it. Use this when changing the
     * limit during a match. Limits sent this way reach the Talon in the order they were set.
     * @param limit Amps
     */
    public void setStatorLimitInBackground(int limit) {
        setStatorLimitNoApply(limit);
        if (deferSetupHere()) return;
        // A copy, so later changes on the robot thread don't race with this send
        final CurrentLimitsConfigs currentLimits = configs.CurrentLimits.clone();
        BACKGROUND_CONFIGS.execute(() -> configTalonFX(() -> motor.getConfigurator().apply(currentLimits)));
    }

    @Override
    public void setStatorLimitNoApply(int limit) {
        configs.CurrentLimits.StatorCurrentLimit = limit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
    }

    @Override
    public void setSupplyLimit(int limit) {
        setSupplyLimitNoApply(limit);
        sendConfig(()-> motor.getConfigurator().apply(configs.CurrentLimits));
    }

    @Override
    public void setSupplyLimitNoApply(int limit) {
        configs.CurrentLimits.SupplyCurrentLimit = limit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    /** Sets both current limits at once, in amps; see {@link #setStatorLimit} and {@link #setSupplyLimit}. */
    public void setCurrentLimit(int statorLimit, int supplyLimit) {
        setCurrentLimitNoApply(statorLimit, supplyLimit);
        sendConfig(()-> motor.getConfigurator().apply(configs.CurrentLimits));
    }

    /** Stores both current limits without sending them. */
    public void setCurrentLimitNoApply(int statorLimit, int supplyLimit) {
        configs.CurrentLimits.StatorCurrentLimit = statorLimit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = supplyLimit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    @Override
    public void setDefaultStatusFrames() {
        setSignalFrequencies(HIGH_PRIORITY_FREQ, HIGH_PRIORITY_FREQ, HIGH_PRIORITY_FREQ, HIGH_PRIORITY_FREQ, HIGH_PRIORITY_FREQ);
    }

	@Override
	public void setPositionStatusFrames() {
        setSignalFrequencies(HIGH_PRIORITY_FREQ, HIGH_PRIORITY_FREQ, MEDIUM_PRIORITY_FREQ, MEDIUM_PRIORITY_FREQ, LOW_PRIORITY_FREQ);
	}

	@Override
	public void setVelocityStatusFrames() {
        setSignalFrequencies(MEDIUM_PRIORITY_FREQ, LOW_PRIORITY_FREQ, HIGH_PRIORITY_FREQ, HIGH_PRIORITY_FREQ, LOW_PRIORITY_FREQ);
	}

	@Override
	public void setFollowerStatusFrames() {
        // Only current and temperature are read from a follower, for the logs
        setSignalFrequencies(LOW_PRIORITY_FREQ, LOW_PRIORITY_FREQ, LOW_PRIORITY_FREQ, MEDIUM_PRIORITY_FREQ, LOW_PRIORITY_FREQ);
	}

    /** Sets how often each signal is reported, in hertz, and turns off every signal not listed. */
    private void setSignalFrequencies(double appliedOutputHz, double positionHz, double velocityHz,
                                      double currentHz, double temperatureHz) {
        pendingFrequencies.clear();
        pendingFrequencies.put(appliedOutput, appliedOutputHz);
        pendingFrequencies.put(position, positionHz);
        pendingFrequencies.put(velocity, velocityHz);
        pendingFrequencies.put(stallCurrent, currentHz);
        pendingFrequencies.put(temperature, temperatureHz);
        // Used by getState(), which the dashboard checks
        pendingFrequencies.put(faults, LOW_PRIORITY_FREQ);
        if (deferSetupHere()) return;

        sendFrequencies(List.of(this));
        pendingFrequencies.clear();
    }

    /**
     * Sends the {@link #pendingFrequencies} of each Talon, with one call per rate for all of them,
     * then turns off every signal not given a rate.
     */
    private static void sendFrequencies(List<NAR_TalonFX> talons) {
        final Map<Double, List<BaseStatusSignal>> signalsByHz = new TreeMap<>();
        final List<NAR_TalonFX> optimized = new ArrayList<>();
        for (NAR_TalonFX talon : talons) {
            if (talon.pendingFrequencies.isEmpty()) continue;
            talon.pendingFrequencies.forEach((signal, hz) ->
                signalsByHz.computeIfAbsent(hz, unused -> new ArrayList<>()).add(signal));
            optimized.add(talon);
        }
        signalsByHz.forEach((hz, signals) -> sendWithRetries(
            () -> BaseStatusSignal.setUpdateFrequencyForAll(hz, signals),
            "Failed to set the " + hz + " Hz status signals"));
        // Phoenix has no single call for this; optimizeBusUtilizationForAll also does one Talon at a time
        for (NAR_TalonFX talon : optimized) {
            talon.configTalonFX(() -> talon.motor.optimizeBusUtilization());
        }
    }

    /**
     * Returns DISCONNECTED if the Talon has not reported recently, PARTIALLY_RUNNING if it is reporting a
     * fault, such as overheating or a dropped CAN connection, and RUNNING otherwise.
     */
    @Override
    public DeviceState getState() {
        // Refreshing a signal the Talon has stopped sending returns an error instead of a value.
        // appliedOutput is always reported, regardless of the status frames.
        if (!appliedOutput.refresh().getStatus().isOK()) return DeviceState.DISCONNECTED;
        // Each bit of the fault field is a separate fault; 0 means none
        if (faults.refresh().getValue() != 0) return DeviceState.PARTIALLY_RUNNING;
        return DeviceState.RUNNING;
    }


    /** Returns the Phoenix 6 {@code TalonFX}, for functionality this class does not cover. */
    @Override
    public TalonFX getMotor() {
        return motor;
    }

    @Override
    public void close() {
        disableSim();
        motor.close();
    }
}
