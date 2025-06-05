package common.hardware.motorcontroller;

import static common.hardware.motorcontroller.MotorControllerConstants.HIGH_PRIORITY_FREQ;
import static common.hardware.motorcontroller.MotorControllerConstants.NEO_STATOR_CurrentLimit;
import static common.hardware.motorcontroller.MotorControllerConstants.NEO_SUPPLY_CurrentLimit;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import common.core.controllers.PIDFFConfig;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

public class NAR_TalonFX extends NAR_Motor {

    private static int numFailedConfigs = 0;

	public static int maximumRetries = 5;

    /**
	 * @return The number of failed configurations of the motor.
	 */
	public static int getNumFailedConfigs() {
		return numFailedConfigs;
	}

    private final TalonFX motor;

    // private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    // private final VoltageConfigs voltageConfigs = new VoltageConfigs();
    // private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    // private final Slot0Configs slot0Configs = new Slot0Configs();
    TalonFXConfiguration configs = new TalonFXConfiguration();

    private final StatusSignal<Double> appliedOutput;
    private final StatusSignal<Current> stallCurrent;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Temperature> temperature;

    public NAR_TalonFX(int deviceNumber, String canbus, PIDFFConfig pidConfig) {
        super(deviceNumber);
        Timer timer = new Timer();
        timer.restart();
        motor = new TalonFX(deviceNumber, canbus);
        timer.stop();
        Log.info("Talon ID " + deviceNumber + " Creation", timer.get());

        appliedOutput = motor.getDutyCycle();
        stallCurrent = motor.getStatorCurrent();
        position = motor.getPosition();
        velocity = motor.getVelocity();
        temperature = motor.getDeviceTemp();

        enableVoltageCompensationNoApply(12);
        setCurrentLimitNoApply(NEO_STATOR_CurrentLimit, NEO_SUPPLY_CurrentLimit);
        apply();
        configPID(pidConfig);
    }

    public NAR_TalonFX(int deviceNumber, String canbus) {
        this(deviceNumber, canbus, new PIDFFConfig());
    }

    public NAR_TalonFX(int deviceNumber) {
        this(deviceNumber, "");
    }
    
    /**
	 * Run the configuration until it succeeds or times out.
	 *
	 * @param config Lambda supplier returning the error state.
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

    @Override
    protected void setVelocity(double rpm, double feedForward) {
        var velocitySetpoint = new VelocityVoltage(rpm);
        velocitySetpoint.FeedForward = feedForward;
        motor.setControl(velocitySetpoint);
    }

    @Override
    protected void setPosition(double rotations, double feedForward) {
        var positionSetpoint = new PositionVoltage(rotations);
        positionSetpoint.FeedForward = feedForward;
        motor.setControl(positionSetpoint);
    }

    @Override
    protected void resetRawPosition(double rotations) {
        motor.setPosition(rotations);
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
		return DCMotor.getKrakenX60(1).withReduction(unitConversionFactor).getTorque(getStallCurrent());
	}

    @Override
    protected double getRawPosition() {
        return position.refresh().getValue().in(Units.Revolution);
    }

    @Override
    protected double getRawVelocity() {
        return velocity.refresh().getValue().in(Units.RotationsPerSecond) * 60.0;
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
        configs.Voltage.PeakReverseVoltage = volts;
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

    public void setCurrentLimit(int statorLimit, int supplyLimit) {
        setCurrentLimitNoApply(statorLimit, supplyLimit);
        configTalonFX(()-> motor.getConfigurator().apply(configs.CurrentLimits));
    }

    public void setCurrentLimitNoApply(int statorLimit, int supplyLimit) {
        configs.CurrentLimits.StatorCurrentLimit = statorLimit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = supplyLimit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    @Override
    public void setDefaultStatusFrames() {
        configTalonFX(()-> appliedOutput.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> stallCurrent.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> velocity.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> position.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> temperature.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()->  motor.optimizeBusUtilization());
    }
    
	@Override
	public void setPositionStatusFrames() {
        configTalonFX(()-> position.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> motor.optimizeBusUtilization());
	}

	@Override
	public void setVelocityStatusFrames() {
        configTalonFX(()-> velocity.setUpdateFrequency(HIGH_PRIORITY_FREQ));
        configTalonFX(()-> motor.optimizeBusUtilization());
	}

	@Override
	public void setFollowerStatusFrames() {
		configTalonFX(()-> motor.optimizeBusUtilization());
	}

    @Override
    public NarwhalDashboard.State getState() {
        return NarwhalDashboard.State.RUNNING;
    }


    public TalonFX getMotor() {
        return motor;
    }

    @Override
    public void close() {
        motor.close();
    }
}
