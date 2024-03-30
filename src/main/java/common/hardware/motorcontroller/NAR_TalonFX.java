package common.hardware.motorcontroller;

import static common.hardware.motorcontroller.MotorControllerConstants.HIGH_PRIORITY_FREQ;
import static common.hardware.motorcontroller.MotorControllerConstants.NEO_CurrentLimit;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import common.core.controllers.PIDFFConfig;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;

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

    private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    private final VoltageConfigs voltageConfigs = new VoltageConfigs();
    private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    private final StatusSignal<Double> appliedOutput;
    private final StatusSignal<Double> stallCurrent;
    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;
    private final StatusSignal<Double> temperature;

    public NAR_TalonFX(int deviceNumber, String canbus, PIDFFConfig pidConfig) {
        super(deviceNumber);
        motor = new TalonFX(deviceNumber, canbus);

        appliedOutput = motor.getDutyCycle();
        stallCurrent = motor.getStatorCurrent();
        position = motor.getPosition();
        velocity = motor.getVelocity();
        temperature = motor.getDeviceTemp();

        enableVoltageCompensation(12);
        setCurrentLimit(NEO_CurrentLimit);
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
    public void configPID(PIDFFConfig config) {
        var PID0 = new Slot0Configs();
        PID0.kP = config.kP;
        PID0.kI = config.kI;
        PID0.kD = config.kD;
        // PID0.kS = config.kS;
        // PID0.kV = config.kV;
        // PID0.kA = config.kA;
        // PID0.kG = config.kG;
        configTalonFX(()-> motor.getConfigurator().apply(PID0));
    }

    @Override
    public void setInverted(boolean inverted) {
        motorOutputConfigs.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        configTalonFX(()-> motor.getConfigurator().apply(motorOutputConfigs));
    }

    @Override
    protected void setPercentOutput(double speed) {
        motor.set(speed);
    }

    @Override
    protected void setVelocity(double rpm, double feedForward) {
        var velocitySetpoint = new VelocityVoltage(rpm, 0, false, feedForward, 0, false, false, false);
        motor.setControl(velocitySetpoint);
    }

    @Override
    protected void setPosition(double rotations, double feedForward) {
        var positionSetpoint = new PositionVoltage(rotations, 0, false, feedForward, 0, false, false, false);
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
        return stallCurrent.refresh().getValue();
    }

    @Override
    protected double getRawPosition() {
        return position.refresh().getValue();
    }

    @Override
    protected double getRawVelocity() {
        return velocity.refresh().getValue() * 60.0;
    }

    @Override
    public double getTemperature() {
        return temperature.refresh().getValue();
    }

    @Override
    protected void setBrakeMode() {
       motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
       configTalonFX(()-> motor.getConfigurator().apply(motorOutputConfigs));
    }

    @Override
    protected void setCoastMode() {
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        configTalonFX(()-> motor.getConfigurator().apply(motorOutputConfigs));
    }

    @Override
    public void enableVoltageCompensation(double volts) {
        voltageConfigs.PeakForwardVoltage = volts;
        voltageConfigs.PeakReverseVoltage = volts;
        configTalonFX(()-> motor.getConfigurator().apply(voltageConfigs));
    }

    @Override
    public void setCurrentLimit(int limit) {
        currentLimitsConfigs.StatorCurrentLimit = limit;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        configTalonFX(()-> motor.getConfigurator().apply(currentLimitsConfigs));
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

    @Override
    public TalonFX getMotor() {
        return motor;
    }

    @Override
    public void close() {
        motor.close();
    }
}
