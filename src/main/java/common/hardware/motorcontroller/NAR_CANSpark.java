package common.hardware.motorcontroller;

import java.util.LinkedList;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.jni.CANBusJNI;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import common.core.controllers.PIDFFConfig;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import static common.hardware.motorcontroller.MotorControllerConstants.*;

/**
 * Team 3128's streamlined {@link SparkBase} class.
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public class NAR_CANSpark extends NAR_Motor {

	private static int numFailedConfigs = 0;

	public static int maximumRetries = 5;

	public static final LinkedList<NAR_CANSpark> instances = new LinkedList<NAR_CANSpark>();

	/**
	 * Flashes all spark max's
	 */
	public static void burnFlashAll() {
		for (final NAR_CANSpark spark : instances) {
			spark.burnFlash();
		}
	}

	/**
	 * @return The number of failed configurations of the motor.
	 */
	public static int getNumFailedConfigs() {
		return numFailedConfigs;
	}

	/**
	 * Team 3128's status frames
	 */
	public enum NAR_SparkMaxConfig {
		DEFAULT(HIGH_PRIORITY, HIGH_PRIORITY, HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		FOLLOWER(HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		POSITION(HIGH_PRIORITY, NO_PRIORITY, HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		VELOCITY(HIGH_PRIORITY, HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		ABSOLUTE(HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, HIGH_PRIORITY, HIGH_PRIORITY);

		public final int status0, status1, status2, status3, status4, status5, status6;

		private NAR_SparkMaxConfig(int status0, int status1, int status2, int status3, int status4, int status5, int status6) {
			this.status0 = status0;
			this.status1 = status1;
			this.status2 = status2;
			this.status3 = status3;
			this.status4 = status4;
			this.status5 = status5;
			this.status6 = status6;
		}
	}
	/**
	 * Type of encoder used
	 */
    public enum EncoderType {
		Relative,
		Absolute
	}

	/**
	 * Type of controller used
	 */
	public enum ControllerType {
		CAN_SPARK_MAX,
		CAN_SPARK_FLEX
	}
	
	private double kP, kI, kD;
	private EncoderType encoderType;
	private RelativeEncoder relativeEncoder;
	private AbsoluteEncoder absoluteEncoder;
	private final SparkClosedLoopController controller;
    private final SparkBase motor;
	private final SparkBaseConfig config;

    /**
	 * Create a new object to control a SPARK motor
	 *
	 * @param deviceNumber The device ID.
	 * @param controllerType The type of controller used, ie. CAN_SPARK_MAX or CAN_SPARK_FLEX
	 * @param encoderType The type of encoder used, ie. relative build in encoder or absolute encoder
	 * 		connected by an adapter.
	 * @param motorType The motor type connected to the controller. Brushless motor wires must be connected
	 *     	to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *     	connected to the Red and Black terminals only.
	 * @param PIDconfig PIDFFConfig containing kP, kI, and kD values.
	 */
    public NAR_CANSpark(int deviceNumber, ControllerType controllerType, MotorType motorType, EncoderType encoderType, PIDFFConfig PIDconfig) {
		super(deviceNumber);
		motor = controllerType == ControllerType.CAN_SPARK_MAX ? new SparkMax(deviceNumber, motorType) : new SparkFlex(deviceNumber, motorType);
		
		config = controllerType == ControllerType.CAN_SPARK_MAX ? new SparkMaxConfig() : new SparkFlexConfig();
		configSpark(()-> motor.setCANTimeout(canSparkMaxTimeout));
		configSpark(()-> motor.clearFaults());


		motor.setCANMaxRetries(0);
		configSpark(()-> motor.clearFaults());
		enableVoltageCompensation(12.0, false);
		setStatorLimit(motorType == MotorType.kBrushless ? NEO_STATOR_CurrentLimit : NEO_STATOR_550CurrentLimit, false);
		setSupplyLimit(motorType == MotorType.kBrushless ? NEO_SUPPLY_CurrentLimit : NEO_SUPPLY_550CurrentLimit, false);

		this.encoderType = encoderType;

		relativeEncoder = motor.getEncoder();
		if (encoderType == EncoderType.Relative) {
			//No clue what this does, but Mechanical Advantage does this so it must be good
			config.encoder.uvwAverageDepth(2);
			config.encoder.uvwMeasurementPeriod(10);
		}
		else {
			config.absoluteEncoder.averageDepth(2);
			config.absoluteEncoder.velocityConversionFactor(60);
		}

		controller = motor.getClosedLoopController();
		configPID(PIDconfig, false);
		configure();
		instances.add(this);
    }

    /**
	 * Create a new object to control a SPARK motor
	 *
	 * @param deviceNumber The device ID.
	 * @param controllerType The type of controller used, ie. CAN_SPARK_MAX or CAN_SPARK_FLEX
     * @param motorType The motor type connected to the controller. Brushless motor wires must be connected
	 *     	to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *     	connected to the Red and Black terminals only.
	 * @param encoderType The type of encoder used, ie. relative build in encoder or absolute encoder
	 * 		connected by an adapter.
	 */
	public NAR_CANSpark(int deviceNumber, ControllerType controllerType, MotorType motorType, EncoderType encoderType) {
		this(deviceNumber, controllerType, motorType, encoderType, new PIDFFConfig());
	}

    /**
	 * Create a new object to control a SPARK motor
	 *
	 * @param deviceNumber The device ID.
	 * @param controllerType The type of controller used, ie. CAN_SPARK_MAX or CAN_SPARK_FLEX
     * @param motorType The motor type connected to the controller. Brushless motor wires must be connected
	 *     	to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *     	connected to the Red and Black terminals only.
	 */
	public NAR_CANSpark(int deviceNumber, ControllerType controllerType, MotorType motorType) {
		this(deviceNumber, controllerType, motorType, EncoderType.Relative);
	}
    
    /**
	 * Create a new object to control a SPARK motor
	 *
	 * @param deviceNumber The device ID.
	 * @param controllerType The type of controller used, ie. CAN_SPARK_MAX or CAN_SPARK_FLEX
	 */
	public NAR_CANSpark(int deviceNumber, ControllerType controllerType) {
		this(deviceNumber, controllerType, MotorType.kBrushless);
	}

	/**
	 * Create a new object to control a SPARK MAX motor
	 *
	 * @param deviceNumber The device ID.
	 */
	public NAR_CANSpark(int deviceNumber) {
		this(deviceNumber, ControllerType.CAN_SPARK_MAX, MotorType.kBrushless);
	}

	/**
	 * Run the configuration until it succeeds or times out.
	 *
	 * @param config Lambda supplier returning the error state.
	 */
	private void configSpark(Supplier<REVLibError> config)
	{
		for (int i = 0; i < maximumRetries; i++)
		{
			if (config.get() == REVLibError.kOk)
			{
				return;
			}
		}
		numFailedConfigs ++;
		Log.info("Motors", "Failed to configure Spark Max " + motor.getDeviceId());
	}

	@Override
	public void apply() {
		configure();
	}

	public void configure() {
		motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void configPID(PIDFFConfig config, boolean apply) {
		this.kP = config.kP;
		this.kI = config.kI;
		this.kD = config.kD;

		this.config.closedLoop
		.p(kP)
		.i(kI)
		.d(kD);

		if (apply) configure();
	}

	/**
	 * Sets the current limit in Amps.
	 *
	 * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
	 * limit. This limit is enabled by default and used for brushless only. This limit is highly
	 * recommended when using the NEO brushless motor.
	 *
	 * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes
	 * that could be enough to cause damage to the motor and controller. This current limit provides a
	 * smarter strategy to deal with high current draws and keep the motor and controller operating in
	 * a safe region.
	 *
	 * @param limit The current limit in Amps.
	 */
	@Override
	public void setStatorLimit(int limit, boolean apply) {
		config.smartCurrentLimit(limit);
		if (apply) configure();
	}


	/**
   * Sets the secondary current limit in Amps.
   *
   * <p>The motor controller will disable the output of the controller briefly if the current limit
   * is exceeded to reduce the current. This limit is a simplified 'on/off' controller. This limit
   * is enabled by default but is set higher than the default Smart Current Limit.
   *
   * <p>The time the controller is off after the current limit is reached is determined by the
   * parameter limitCycles, which is the number of PWM cycles (20kHz). The recommended value is the
   * default of 0 which is the minimum time and is part of a PWM cycle from when the over current is
   * detected. This allows the controller to regulate the current close to the limit value.
   *
   * <p>The total time is set by the equation <code>
   * t = (50us - t0) + 50us * limitCycles
   * t = total off time after over current
   * t0 = time from the start of the PWM cycle until over current is detected
   * </code>
   *
   * @param limit The current limit in Amps.
   */
	@Override
	public void setSupplyLimit(int limit, boolean apply) {
		config.secondaryCurrentLimit(limit);
		if (apply) configure();
	}

    /**
	 * Set the rate of transmission for periodic frames from the SPARK MAX
	 *
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to change the default rates.
	 * 
	 * <p><b>Status 0</b>: Applied Output, Faults, Sticky Faults, isFollower
	 * <p><b>Status 1</b>: Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
	 * <p><b>Status 2</b>: Motor Position
	 * <p><b>Status 3</b>: Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
	 * <p><b>Status 4</b>: Alternate Encoder Velocity
	 * <p><b>Status 5</b>: Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Absolute Angle
	 * <p><b>Status 6</b>: Duty Cycle Absolute Encoder Velocity, Duty Cycle Absolute Encoder Frequency
	 * 
	 * @param frame Which type of {@link PeriodicFrame} to change the period of
	 * @param periodMs Period in ms for the given frame.
	 */
	public void setPeriodicFramePeriod(PeriodicFrame frame, int periodMs, boolean apply) {
		config.signals.primaryEncoderPositionPeriodMs(periodMs);
		if (apply) configure();
	}

	/**
	 * Set the rate of transmission for periodic frames from the SPARK MAX
	 *
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to set to team 3128's preset rates.
	 */
	public void setStatusFrames(NAR_SparkMaxConfig config) {
		setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.status0, false);
		setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.status1, false);
		setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.status2, false);
		setPeriodicFramePeriod(PeriodicFrame.kStatus3, config.status3, false);
		setPeriodicFramePeriod(PeriodicFrame.kStatus4, config.status4, false);
		setPeriodicFramePeriod(PeriodicFrame.kStatus5, config.status5, false);
		setPeriodicFramePeriod(PeriodicFrame.kStatus6, config.status6, false);
		apply();
	}

	@Override
	public void setDefaultStatusFrames() {
		setStatusFrames(NAR_SparkMaxConfig.DEFAULT);
	}

	@Override
	public void setPositionStatusFrames() {
		setStatusFrames(NAR_SparkMaxConfig.POSITION);
	}

	@Override
	public void setVelocityStatusFrames() {
		setStatusFrames(NAR_SparkMaxConfig.VELOCITY);
	}

	@Override
	public void setFollowerStatusFrames() {
		setStatusFrames(NAR_SparkMaxConfig.FOLLOWER);
	}

	/**
	 * Burns all settings to flash; stores settings between power cycles
	 */
	public REVLibError burnFlash() {
		 if (RobotBase.isSimulation()) return REVLibError.kOk;

		Timer.delay(0.5);
		REVLibError status = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		Timer.delay(0.5);
		if (status == REVLibError.kOk) {
			Log.info("Motors", "Burned flash for Spark Max " + motor.getDeviceId());
		} else {
			Log.unusual("Motors", "Failed to burn flash for Spark Max " + motor.getDeviceId());
		}

		return status;
	}

	/**
     * Sets a motor's output based on the leader's
     * @param leader The motor to follow
	 * @param invert Whether or not to invert motor output
     */
	public void follow(NAR_CANSpark leader, boolean invert, boolean apply) {
		config.follow(leader.motor, invert);
		if (apply) configure();
	}

	@Override
	public void follow(NAR_Motor leader) {
		if (leader instanceof NAR_CANSpark) {
			follow((NAR_CANSpark) leader, false);
			return;
		}
		super.follow(leader);
	}

	@Override
	public void setInverted(boolean inverted, boolean apply) {
		config.inverted(inverted);
		if (apply) configure();
	}

    @Override
    public void setPercentOutput(double speed) {
        controller.setReference(speed, ControlType.kDutyCycle);
    }

    @Override
    public void setVelocity(double rpm, double feedForward) {
        controller.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedForward);
    }

    @Override
    public void setPosition(double rotations, double feedForward) {
        controller.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
    }

    @Override
    public double getAppliedOutput() {
        return motor.getAppliedOutput();
    }

	@Override
	public double getStallCurrent() {
		return motor.getOutputCurrent();
	}

	@Override
	public double getTorque() {
		if(motor instanceof SparkFlex) return DCMotor.getNEO(1).withReduction(unitConversionFactor).getTorque(getStallCurrent());
		return DCMotor.getNeoVortex(1).withReduction(unitConversionFactor).getTorque(getStallCurrent());
	}
	
	@Override
	public void resetRawPosition(double rotations) {
		if (encoderType == EncoderType.Relative) relativeEncoder.setPosition(rotations);
	}

    @Override
    public double getRawPosition() {
        return encoderType == EncoderType.Relative ? relativeEncoder.getPosition() : absoluteEncoder.getPosition();
    }

    @Override
    public double getRawVelocity() {
        return encoderType == EncoderType.Relative ? relativeEncoder.getVelocity() : absoluteEncoder.getVelocity();
    }

	@Override
	public double getTemperature() {
		return motor.getMotorTemperature();
	}

	@Override
	public void enableVoltageCompensation(double volts, boolean apply) {
		config.voltageCompensation(volts);
		if (apply) configure();
	}

	@Override
	public void setBrakeMode(boolean apply) {
		config.idleMode(IdleMode.kBrake);
		if (apply) configure();
	}

	@Override
	public void setCoastMode(boolean apply) {
		config.idleMode(IdleMode.kCoast);
		if (apply) configure();
	}

	@Override
    public SparkBase getMotor() {
        return motor;
    }

	public Faults getAllFaults() {
		return motor.getFaults();
	}

	//TODO: Eval necessity
	// public boolean getFault(int fault) {
	// 	return motor.getFault(fault);
	// }

	/**
	 * Returns motor and motor controller functionality.
	 * @return State of the motor controller and motor.
	 */
	public State getState() {
		//To do add check for motors
		if(!motor.hasActiveFault() 
			&& !motor.hasStickyFault()
			&& motor.getLastError() == REVLibError.kOk) {
			return State.RUNNING;
		}
		return State.DISCONNECTED;
	}

	/**
     * Closes the spark.
     */
    @Override
    public void close() {
        motor.close();
    }
}
