package common.hardware.motorcontroller;

import static common.hardware.motorcontroller.MotorControllerConstants.HIGH_PRIORITY_FREQ;
import static common.hardware.motorcontroller.MotorControllerConstants.LOW_PRIORITY_FREQ;
import static common.hardware.motorcontroller.MotorControllerConstants.MEDIUM_PRIORITY_FREQ;
import static common.hardware.motorcontroller.MotorControllerConstants.NEO_STATOR_CurrentLimit;
import static common.hardware.motorcontroller.MotorControllerConstants.NEO_SUPPLY_CurrentLimit;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
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

import java.util.function.Supplier;

/**
 * A CTRE TalonFX motor controller, which drives Kraken X60, Kraken X44 and Falcon 500 motors,
 * using Phoenix 6.
 *
 * <p>All settings are kept in one {@code TalonFXConfiguration}. Changing a setting updates that object and
 * sends the changed section to the Talon, retrying if the Talon does not confirm it.
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

    private static int numFailedConfigs = 0;

	/** The number of attempts to send a setting before failing. */
	public static int maximumRetries = 5;

    /**
	 * @return The number of settings, across every Talon, that failed after all retries.
	 */
	public static int getNumFailedConfigs() {
		return numFailedConfigs;
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
        apply();
        configPID(pidConfig);
    }

    /**
     * @param deviceNumber The Talon's CAN ID
     * @param canbus The CAN bus name: {@code "rio"} for the roboRIO's bus, or a CANivore's name
     */
    public NAR_TalonFX(int deviceNumber, String canbus) {
        this(deviceNumber, canbus, new PIDFFConfig());
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
		for (int i = 0; i < maximumRetries; i++)
		{
			if (config.get() == StatusCode.OK)
			{
				return;
			}
		}
		numFailedConfigs ++;
		Log.info("Motors", "Failed to configure Talon FX " + motor.getDeviceID());
	}

    @Override
    public void apply() {
        configTalonFX(() -> motor.getConfigurator().apply(configs));
    }

    @Override
    public void configPID(PIDFFConfig config) {
        configPIDNoApply(config);
        configTalonFX(()-> motor.getConfigurator().apply(configs.Slot0));
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
        configTalonFX(()-> motor.getConfigurator().apply(configs.Slot0));
    }

    /**
     * Arm_Cosine requires the Talon's position in arm rotations, so the Talon divides by this ratio.
     * The rest of this class still uses rotor rotations.
     */
    @Override
    protected void setMechanismRatio(double motorRotationsPerMechanismRotation) {
        mechanismRatio = motorRotationsPerMechanismRotation;
        configs.Feedback.SensorToMechanismRatio = mechanismRatio;
        configTalonFX(()-> motor.getConfigurator().apply(configs.Feedback));
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
        configTalonFX(()-> motor.getConfigurator().apply(configs.MotorOutput));
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
        configTalonFX(()-> motor.getConfigurator().apply(configs.MotorOutput));
    }

    @Override
    protected void setBrakeModeNoApply() {
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    @Override
    protected void setCoastMode() {
        setCoastModeNoApply();
        configTalonFX(()-> motor.getConfigurator().apply(configs.MotorOutput));
    }

    @Override
    protected void setCoastModeNoApply() {
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    @Override
    public void enableVoltageCompensation(double volts) {
        enableVoltageCompensationNoApply(volts);
        configTalonFX(()-> motor.getConfigurator().apply(configs.Voltage));
    }

    @Override
    public void enableVoltageCompensationNoApply(double volts) {
        configs.Voltage.PeakForwardVoltage = volts;
        configs.Voltage.PeakReverseVoltage = -volts;
    }

    @Override
    public void setStatorLimit(int limit) {
        setStatorLimitNoApply(limit);
        configTalonFX(()-> motor.getConfigurator().apply(configs.CurrentLimits));
    }

    @Override
    public void setStatorLimitNoApply(int limit) {
        configs.CurrentLimits.StatorCurrentLimit = limit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
    }

    @Override
    public void setSupplyLimit(int limit) {
        setSupplyLimitNoApply(limit);
        configTalonFX(()-> motor.getConfigurator().apply(configs.CurrentLimits));
    }

    @Override
    public void setSupplyLimitNoApply(int limit) {
        configs.CurrentLimits.SupplyCurrentLimit = limit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    /** Sets both current limits at once, in amps; see {@link #setStatorLimit} and {@link #setSupplyLimit}. */
    public void setCurrentLimit(int statorLimit, int supplyLimit) {
        setCurrentLimitNoApply(statorLimit, supplyLimit);
        configTalonFX(()-> motor.getConfigurator().apply(configs.CurrentLimits));
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
        configTalonFX(()-> appliedOutput.setUpdateFrequency(appliedOutputHz));
        configTalonFX(()-> position.setUpdateFrequency(positionHz));
        configTalonFX(()-> velocity.setUpdateFrequency(velocityHz));
        configTalonFX(()-> stallCurrent.setUpdateFrequency(currentHz));
        configTalonFX(()-> temperature.setUpdateFrequency(temperatureHz));
        // Used by getState(), which the dashboard checks
        configTalonFX(()-> faults.setUpdateFrequency(LOW_PRIORITY_FREQ));
        configTalonFX(()-> motor.optimizeBusUtilization());
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
    public TalonFX getMotor() {
        return motor;
    }

    @Override
    public void close() {
        disableSim();
        motor.close();
    }
}
