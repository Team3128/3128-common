package common.hardware.motorcontroller;

import static common.hardware.motorcontroller.MotorControllerConstants.HIGH_PRIORITY_FREQ;
import static common.hardware.motorcontroller.MotorControllerConstants.NEO_STATOR_CurrentLimit;
import static common.hardware.motorcontroller.MotorControllerConstants.NEO_SUPPLY_CurrentLimit;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import common.core.controllers.PIDFFConfig;
import common.core.misc.NAR_Robot;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Team 3128's motor class replacement for {@link MotorController}
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public class NAR_Motor implements AutoCloseable {

    /**
     * Store conversion factors for motor
     */
    public static class MotorConfig {
        public final double distanceFactor;
        public final double timeFactor;
        public final int supplyLimit;
        public final int statorLimit;
        public final double voltageCompensation;
        public final boolean inverted;
        public final Neutral mode;
        public final StatusFrames statusFrames;

        /**
         * Creates a new motor config
         * @param distanceFactor Factor to multiply distance units by (per rotations) ie. input circumference for distance
         * @param timeFactor Factor to multiply time units by (per minutes) ie. use 60 for seconds, 1 for minutes
         * @param statorLimit The currentLimit for the stator
         * @param supplyLimit The currentLimit for the motor controller
         * @param voltageCompensation The voltage compensation for the motor
         * @param inverted Whether or not the motor is inverted.
         * @param mode Neutral mode for the motor.
         * @param statusFrames Status frames for the motor.
         */
        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, int supplyLimit, double voltageCompensation, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this.distanceFactor = distanceFactor;
            this.timeFactor = timeFactor;
            this.statorLimit = statorLimit;
            this.supplyLimit = supplyLimit;
            this.voltageCompensation = voltageCompensation;
            this.inverted = inverted;
            this.mode = mode;
            this.statusFrames = statusFrames;
        }

        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, double voltageCompensation, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this(distanceFactor, timeFactor, statorLimit, 120, voltageCompensation, inverted, mode, statusFrames);
        }

        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this(distanceFactor, timeFactor, statorLimit, 120, 12, inverted, mode, statusFrames);
        }

        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, boolean inverted, Neutral mode) {
            this(distanceFactor, timeFactor, statorLimit, 120, 12, inverted, mode, StatusFrames.DEFAULT);
        }

        public MotorConfig follower() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, this.inverted, this.mode, StatusFrames.FOLLOWER);
        }

        @Override
        public String toString() {
            String output = "Motor Config";
            output.concat("\n Distance Factor: " + distanceFactor + ". \tEvery one true rotation of the motor will be measured as " + distanceFactor + " rotation(s).");
            output.concat("\n Time Factor: " + timeFactor + ". \tMotor velocity will be measured in units equivalent to " + (1 / timeFactor) + " rotations per minute.");
            output.concat("\n Stator Limit: " + statorLimit + ". \tThe maximum current applied to the stator is " + statorLimit + " amps. Max torque limited.");
            output.concat("\n Supply Limit: " + supplyLimit + ". \tThe maximum current applied to the motor is " + supplyLimit + " amps.");
            output.concat("\n Voltage Compensation: " + voltageCompensation + ". \tThe maximum voltage applied to the motor is " + voltageCompensation + " volts.");
            output.concat("\n Inversion: " + inverted + ". \t" + (inverted ? ("Clockwise is positive angular displacement.") : ("Counter-Clockwise is positive angular displacement.")));
            output.concat("\n Neutral Mode: " + mode.name() + ". \tWhen no output is applied the motor will " + mode.name().toLowerCase() + ".");
            output.concat("\n Status Frames: " + statusFrames.name() + ". \tThe motor will send status frames consistent with a " + statusFrames.name().toLowerCase() + " motor.");
            return output;
        }
    }

    public enum StatusFrames {
        DEFAULT,
        POSITION,
        VELOCITY,
        FOLLOWER;
    }

    /**
     * Motor control modes
     */
    public enum Control {
        PercentOutput,
        Velocity,
        Position;
    }


    /**
     * Motor states when no voltage is applied
     */
    public enum Neutral {
        BRAKE,
        COAST
    }

    private static final HashSet<NAR_Motor> leaders = new HashSet<NAR_Motor>();

    private static int numFailedConfigs = 0;

    public static int maximumRetries = 5;

    public static int getNumFailedConfigs() {
        return numFailedConfigs;
    }

    static {
        NAR_Robot.addPeriodic(()-> {
            for (final NAR_Motor leader : leaders) {
                final double output = leader.getAppliedOutput();
                for (final NAR_Motor follower : leader.followers) {
                    follower.set(output);
                }
            }
        }, 0.1);
    }

    private final LinkedList<NAR_Motor> followers = new LinkedList<NAR_Motor>();
    private final int id;
    private double prevValue = 0;
    private Control prevMode = Control.PercentOutput;
    private double prevFeedForward = 0;
    private double minInput;
    private double maxInput;
    private boolean isContinuous = false;
    private boolean useFOC = false;
    private double unitConversionFactor = 1;
    private double timeConversionFactor = 1;

    private final TalonFX motor;
    private final TalonFXConfiguration configs = new TalonFXConfiguration();

    private final StatusSignal<Double> appliedOutput;
    private final StatusSignal<Current> stallCurrent;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Temperature> temperature;

    public NAR_Motor(int id, String canbus, PIDFFConfig pidConfig) {
        this.id = id;
        motor = new TalonFX(id, new CANBus(canbus));

        appliedOutput = motor.getDutyCycle();
        stallCurrent = motor.getStatorCurrent();
        position = motor.getPosition();
        velocity = motor.getVelocity();
        temperature = motor.getDeviceTemp();

        enableVoltageCompensationNoApply(12);
        setStatorLimitNoApply(NEO_STATOR_CurrentLimit);
        setSupplyLimitNoApply(NEO_SUPPLY_CurrentLimit);
        configPIDNoApply(pidConfig);
        apply();
    }

    public NAR_Motor(int id, String canbus) {
        this(id, canbus, new PIDFFConfig());
    }

    public NAR_Motor(int id) {
        this(id, "");
    }

    private void configTalonFX(Supplier<StatusCode> config) {
        for (int i = 0; i < maximumRetries; i++) {
            if (config.get() == StatusCode.OK) return;
        }
        numFailedConfigs++;
        Log.info("Motors", "Failed to configure Talon FX " + id);
    }

    public void enableFOC(boolean enable) {
        useFOC = enable;
    }

    /**
     * Sets motor output power in volts
     * @param volts The voltage of the motor from -12 to 12
     */
    public void setVolts(double volts) {
        set(volts / 12.0);
    }

    /**
     * Sets motor output power
     * @param output The speed of the motor from -1 to 1
     */
    public void set(double output) {
        set(output, Control.PercentOutput);
    }

    /**
     * Sets the motor state
     * @param value Output of motor dependent on control mode
     * @param mode Type of control mode
     */
    public void set(double value, Control mode) {
        set(value, mode, 0);
    }

    /**
     * Sets the motor state with feedforward
     * @param value Output of motor dependent on control mode
     * @param mode Type of control mode
     * @param feedForward Feedforward of motor measured in volts
     */
    public void set(double value, Control mode, double feedForward) {
        if (value == prevValue && mode == prevMode && feedForward == prevFeedForward) return;
        prevValue = value;
        prevMode = mode;
        prevFeedForward = feedForward;
        switch(mode) {
            case PercentOutput:
                setPercentOutput(MathUtil.clamp(value, -1, 1));
                break;
            case Velocity:
                setVelocity(value / unitConversionFactor * timeConversionFactor, feedForward);
                break;
            case Position:
                if (isContinuous) {
                    final double position = getRawPosition() * unitConversionFactor;
                    final double errorBound = (maxInput - minInput) / 2.0;
                    final double error = convertInput(value) - convertInput(position);
                    final double delta = MathUtil.inputModulus(error, -errorBound, errorBound);
                    setPosition((position + delta) / unitConversionFactor, feedForward);
                    break;
                }
                setPosition(value / unitConversionFactor, feedForward);
                break;
        }
    }

    public void setVelocityFOC2(double setpoint, double feedforward) {
        var velocitySetpoint = new VelocityVoltage(setpoint / unitConversionFactor * timeConversionFactor / 60)
            .withEnableFOC(true)
            .withFeedForward(feedforward);
        motor.setControl(velocitySetpoint);
    }

    /**
     * Configures all major motor settings.
     * @param config Motor settings.
     */
    public void configMotor(MotorConfig config) {
        setUnitConversionFactor(config.distanceFactor);
        setTimeConversionFactor(config.timeFactor);
        setInvertedNoApply(config.inverted);
        setStatorLimitNoApply(config.statorLimit);
        setSupplyLimitNoApply(config.supplyLimit);
        enableVoltageCompensationNoApply(config.voltageCompensation);
        setNeutralModeNoApply(config.mode);
        switch(config.statusFrames) {
            case DEFAULT:
                setDefaultStatusFrames();
                break;
            case POSITION:
                setPositionStatusFrames();
                break;
            case VELOCITY:
                setVelocityStatusFrames();
                break;
            case FOLLOWER:
                setFollowerStatusFrames();
                break;
        }

        apply();

        Log.debug(Log.Type.MOTOR, "Motor (" + this.id + ")", config.toString());
    }

    public void apply() {
        configTalonFX(() -> motor.getConfigurator().apply(configs));
    }

    /**
	 * Set the PID values for the controller.
	 * @param config PIDFFConfig containing kP, kI, and kD values.
	 */
    public void configPID(PIDFFConfig config) {
        configPIDNoApply(config);
        configTalonFX(()-> motor.getConfigurator().apply(configs.Slot0));
    }

    public void configPIDNoApply(PIDFFConfig config) {
        configs.Slot0.kP = config.kP;
        configs.Slot0.kI = config.kI;
        configs.Slot0.kD = config.kD;
    }

    /**
     * Wraps a measurement value to the min and max input
     * @param value A measurement value
     * @return The measurement value wrapped by the min and max input
     */
    private double convertInput(double value) {
        return isContinuous ? MathUtil.inputModulus(value, minInput, maxInput) : value;
    }

    /** Enables continuous input.
    *
    * <p>Rather then using the max and min input range as constraints, the motor considers them to be the
    * same point and automatically calculates the shortest route to the setpoint.
    *
    * @param minInput The minimum value expected from the input.
    * @param maxInput The maximum value expected from the input.
    */
    public void enableContinuousInput(double minInput, double maxInput) {
        this.minInput = Math.min(minInput, maxInput);
        this.maxInput = Math.max(minInput, maxInput);
        isContinuous = true;
    }

    /**
     * Changes the units the motor measures position in, ie. rotations degrees
     * @param conversionFactor Conversion factor to change position units
     */
    public void setUnitConversionFactor(double conversionFactor) {
        this.unitConversionFactor = conversionFactor;
    }

    /**
     * Changes the units the motor measures time/velocity in, ie. rotations per minute rotations per second
     * @param conversionFactor Conversion factor to change time units
     */
    public void setTimeConversionFactor(double conversionFactor) {
        this.timeConversionFactor = conversionFactor;
    }

    /**
     * Resets the motor position
     * @param position Motor position, default units - rotations
     */
    public void resetPosition(double position) {
        resetRawPosition(position / unitConversionFactor);
    }

    /**
     * Sets the motor inverted
     * @param inverted Inverts the motor
     */
    public void setInverted(boolean inverted) {
        setInvertedNoApply(inverted);
        configTalonFX(()-> motor.getConfigurator().apply(configs.MotorOutput));
    }

    public void setInvertedNoApply(boolean inverted) {
        configs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    }

    /**
     * Sets motor output power
     * @param speed The speed of the motor from -1 to 1
     */
    private void setPercentOutput(double speed) {
        motor.set(speed);
    }

    /**
     * Sets the motor output based on its velocity
     * @param rpm Velocity of the motor in RPM
     * @param feedForward Feedforward component measured in volts
     */
    private void setVelocity(double rpm, double feedForward) {
        var velocitySetpoint = new VelocityVoltage(rpm / 60)
            .withEnableFOC(useFOC)
            .withFeedForward(feedForward);
        motor.setControl(velocitySetpoint);
    }

    /**
     * Sets motor output based on its position
     * @param rotations Position of the motor in rotations
     * @param feedForward Feedforward component measured in volts
     */
    private void setPosition(double rotations, double feedForward) {
        var positionSetpoint = new PositionVoltage(rotations)
            .withEnableFOC(useFOC)
            .withFeedForward(feedForward);
        motor.setControl(positionSetpoint);
    }

    /**
     * Resets the motor position
     * @param rotations Number of rotations
     */
    private void resetRawPosition(double rotations) {
        motor.setPosition(rotations);
    }

    /**
     * Gets the motors current output
     * @return Double measuring percent output from -1 to 1
     */
    public double getAppliedOutput() {
        return appliedOutput.refresh().getValue();
    }

    /**
     * Returns the current going to the motor, increasing values means the motor is stalling
     * @return Double measuring the stall current of the motor
     */
    public double getStallCurrent() {
        return stallCurrent.refresh().getValue().in(Units.Amp);
    }

    /**
     * Returns the current motor position, default unit - rotations
     * @return Double measuring motor position
     */
    public double getPosition() {
        final double position = getRawPosition() * unitConversionFactor;
        return convertInput(position);
    }

    /**
     * Returns the current motor velocity, default unit - RPM
     * @return Double measuring motor velocity
     */
    public double getVelocity() {
        return getRawVelocity() * unitConversionFactor / timeConversionFactor;
    }

    /**
     * Returns the motor position in rotations
     * @return Double measuring motor position
     */
    private double getRawPosition() {
        return position.refresh().getValue().in(Units.Revolution);
    }

    /**
     * Returns the current motor velocity in RPM
     * @return Double measuring motor velocity
     */
    private double getRawVelocity() {
        return velocity.refresh().getValue().in(Units.RotationsPerSecond) * 60.0;
    }

    /**
     * Returns the Motor's temperature in celsius.
     * @return The temperature in celsius.
     */
    public double getTemperature() {
        return temperature.refresh().getValue().in(Units.Celsius);
    }

    /**
     * Sets a motor's output based on the leader's
     * @param leader The motor to follow
     */
    public void follow(NAR_Motor leader) {
        leader.followers.add(this);
        leaders.add(leader);
    }

    /**
     * Sets a motor's output based on the leader's
     * @param leader The motor to follow
     * @param config Motor settings
     */
    public void follow(NAR_Motor leader, MotorConfig config) {
        leader.followers.add(this);
        leaders.add(leader);
        configMotor(config.follower());
    }

    /**
     * Sets the motor's idle mode, its behavior when no voltage is applied
     * Followers will follow the leader's idle mode
     * @param mode Type of idle mode
     */
    public void setNeutralMode(Neutral mode) {
        setNeutralModeNoApply(mode);
        configTalonFX(()-> motor.getConfigurator().apply(configs.MotorOutput));
        followers.forEach(follower -> follower.setNeutralMode(mode));
    }

    public void setNeutralModeNoApply(Neutral mode) {
        configs.MotorOutput.NeutralMode = mode == Neutral.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        followers.forEach(follower -> follower.setNeutralModeNoApply(mode));
    }

    /**
     * Sets voltage compensation, keeps output consistent when battery is above x volts
     * @param volts The max volts the motor goes too
     */
    public void enableVoltageCompensation(double volts) {
        enableVoltageCompensationNoApply(volts);
        configTalonFX(()-> motor.getConfigurator().apply(configs.Voltage));
    }

    public void enableVoltageCompensationNoApply(double volts) {
        configs.Voltage.PeakForwardVoltage = volts;
        configs.Voltage.PeakReverseVoltage = -volts;
    }

    /**
	 * Sets the stator limit in Amps.
	 * @param limit The current limit in Amps.
	 */
    public void setStatorLimit(int limit) {
        setStatorLimitNoApply(limit);
        configTalonFX(()-> motor.getConfigurator().apply(configs.CurrentLimits));
    }

    public void setStatorLimitNoApply(int limit) {
        configs.CurrentLimits.StatorCurrentLimit = limit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
    }

    /**
	 * Sets the supply limit in Amps.
	 * @param limit The current limit in Amps.
	 */
    public void setSupplyLimit(int limit) {
        setSupplyLimitNoApply(limit);
        configTalonFX(()-> motor.getConfigurator().apply(configs.CurrentLimits));
    }

    public void setSupplyLimitNoApply(int limit) {
        configs.CurrentLimits.SupplyCurrentLimit = limit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    /**
	 * Returns motor and motor controller functionality.
	 * @return State of the motor controller and motor.
	 */
    public NarwhalDashboard.State getState() {
        return NarwhalDashboard.State.RUNNING;
    }

    /**
     * Set the status frame rate to Team 3128's defaults
     */
    public void setDefaultStatusFrames() {
        configTalonFX(()-> appliedOutput.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> stallCurrent.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> velocity.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> position.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> temperature.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> motor.optimizeBusUtilization());
    }

    /**
     * Set the status frame rate to Team 3128's for a position motor
     */
    public void setPositionStatusFrames() {
        configTalonFX(()-> appliedOutput.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> position.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> motor.optimizeBusUtilization());
    }

    /**
     * Set the status frame rate to Team 3128's for a velocity motor
     */
    public void setVelocityStatusFrames() {
        configTalonFX(()-> velocity.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> stallCurrent.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> motor.optimizeBusUtilization());
    }

    /**
     * Set the status frame rate to Team 3128's for a follower motor
     */
    public void setFollowerStatusFrames() {
        configTalonFX(()-> motor.optimizeBusUtilization());
    }

    /**
     * Returns the motor object controlling the motion
     * @return The motor ie. SparkMax, TalonFX
     */
    public TalonFX getMotor() {
        return motor;
    }

    @Override
    public void close() {
        motor.close();
    }
}
