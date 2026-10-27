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
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.sim.SparkMaxSim;

import common.core.controllers.PIDFFConfig;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
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
	private SparkMaxSim maxSim;
	private SparkFlexSim flexSim;
	private SparkAbsoluteEncoderSim absoluteSim;
	/** Absolute encoder rotations when the sim model was at zero. */
	private double absoluteSimOffset = 0;

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
		enableVoltageCompensationNoApply(12.0);
		setStatorLimitNoApply(motorType == MotorType.kBrushless ? NEO_STATOR_CurrentLimit : NEO_STATOR_550CurrentLimit);
		setSupplyLimitNoApply(motorType == MotorType.kBrushless ? NEO_SUPPLY_CurrentLimit : NEO_SUPPLY_550CurrentLimit);

		this.encoderType = encoderType;

		relativeEncoder = motor.getEncoder();
		if (encoderType == EncoderType.Relative) {
			//No clue what this does, but Mechanical Advantage does this so it must be good
			config.encoder.uvwAverageDepth(2);
			config.encoder.uvwMeasurementPeriod(10);
		}
		else {
			absoluteEncoder = motor.getAbsoluteEncoder();
			config.absoluteEncoder.averageDepth(2);
			// Read in RPM, like the built-in encoder
			config.absoluteEncoder.velocityConversionFactor(60);
			// Run the Spark's own control loop on the absolute encoder too
			config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
		}

		controller = motor.getClosedLoopController();
		configPIDNoApply(PIDconfig);
		// configure();
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
	public void configPID(PIDFFConfig config) {
		configPIDNoApply(config);
		configure();
	}

	@Override
	public void configPIDNoApply(PIDFFConfig config) {
		this.kP = config.kP;
		this.kI = config.kI;
		this.kD = config.kD;

		this.config.closedLoop
		.p(kP)
		.i(kI)
		.d(kD);
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
	public void setStatorLimit(int limit) {
		setStatorLimitNoApply(limit);
		configure();
	}

	@Override
	public void setStatorLimitNoApply(int limit) {
		config.smartCurrentLimit(limit);
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
	public void setSupplyLimit(int limit) {
		setSupplyLimitNoApply(limit);
		configure();
	}

	@Override
	public void setSupplyLimitNoApply(int limit) {
		config.secondaryCurrentLimit(limit);
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
	public void setPeriodicFramePeriod(PeriodicFrame frame, int periodMs) {
		setPeriodicFramePeriodNoApply(frame, periodMs);
		configure();
	}

	public void setPeriodicFramePeriodNoApply(PeriodicFrame frame, int periodMs) {
		config.signals.primaryEncoderPositionPeriodMs(periodMs);
	}

	/**
	 * Set the rate of transmission for periodic frames from the SPARK MAX
	 *
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to set to team 3128's preset rates.
	 */
	public void setStatusFrames(NAR_SparkMaxConfig config) {
		setPeriodicFramePeriodNoApply(PeriodicFrame.kStatus0, config.status0);
		setPeriodicFramePeriodNoApply(PeriodicFrame.kStatus1, config.status1);
		setPeriodicFramePeriodNoApply(PeriodicFrame.kStatus2, config.status2);
		setPeriodicFramePeriodNoApply(PeriodicFrame.kStatus3, config.status3);
		setPeriodicFramePeriodNoApply(PeriodicFrame.kStatus4, config.status4);
		setPeriodicFramePeriodNoApply(PeriodicFrame.kStatus5, config.status5);
		setPeriodicFramePeriodNoApply(PeriodicFrame.kStatus6, config.status6);
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
	public void follow(NAR_CANSpark leader, boolean invert) {
		followNoApply(leader, invert);
		configure();
	}

	public void followNoApply(NAR_CANSpark leader, boolean invert) {
		config.follow(leader.motor, invert);
	}

	@Override
	protected void configNativeClosedLoop(Control mode, double kP, double kI, double kD,
										  double kS, double kV, double kG, double motorRotationsPerArmRotation) {
		final boolean arm = motorRotationsPerArmRotation > 0;
		config.closedLoop.p(kP).i(kI).d(kD);
		config.closedLoop.feedForward
			.kS(kS)
			.kV(kV)
			.kG(arm ? 0 : kG)
			.kCos(arm ? kG : 0)
			// Converts encoder rotations to arm rotations
			.kCosRatio(arm ? 1 / motorRotationsPerArmRotation : 1);
		configure();
	}

	// REV closed loops run on rotor rotations and RPM (the encoders have no conversion factor),
	// output a fraction of the 12 V compensation voltage, and run every 1 ms without scaling kI and kD
	// by the period, matching WPILib SysId's REV presets. Feedforward gains are in volts.
	@Override
	protected double getNativePositionRotations() {
		return 1;
	}

	@Override
	protected double getNativeVelocityRPM() {
		return 1;
	}

	@Override
	protected double getNativeOutputVolts() {
		return 12;
	}

	@Override
	protected double getNativeTimeSeconds() {
		return 0.001;
	}

	@Override
	public boolean followOnController(NAR_Motor leader, boolean opposeLeader) {
		if (!(leader instanceof NAR_CANSpark spark)) return false;
		follow(spark, opposeLeader);
		return true;
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
	public void setInverted(boolean inverted) {
		setInvertedNoApply(inverted);
		configure();
	}

	@Override
	public void setInvertedNoApply(boolean inverted) {
		config.inverted(inverted);
	}

    @Override
    public void setPercentOutput(double speed) {
        controller.setSetpoint(speed, ControlType.kDutyCycle);
    }

    @Override
    public void setVelocity(double rpm, double feedForward) {
        controller.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedForward);
    }

    @Override
    public void setPosition(double rotations, double feedForward) {
        controller.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
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
		// A Spark Flex drives a Neo Vortex; a Spark Max is assumed to drive a NEO
		final DCMotor neo = motor instanceof SparkFlex ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1);
		return neo.getTorque(getStallCurrent());
	}
	
	@Override
	public void resetRawPosition(double rotations) {
		if (encoderType == EncoderType.Relative) relativeEncoder.setPosition(rotations);
		// An absolute encoder already knows where it is. In simulation it doesn't, so remember where it starts.
		else if (RobotBase.isSimulation()) {
			absoluteSimOffset = rotations - getSimRotorRotations() / motorRotationsPerSensorRotation;
		}
	}

    @Override
    public double getRawPosition() {
        return encoderType == EncoderType.Relative ? relativeEncoder.getPosition() : absoluteEncoder.getPosition();
    }

    @Override
    public double getRawVelocity() {
        return encoderType == EncoderType.Relative ? relativeEncoder.getVelocity() : absoluteEncoder.getVelocity();
    }

	/**
	 * Follows REV's SparkSim pattern. The built-in encoder has no conversion factor, so iterate()
	 * takes rotor RPM. An absolute encoder is moved by the motor's motion divided by
	 * {@link #setMotorRotationsPerSensorRotation(double)}.
	 */
	@Override
	protected void simulationStep(double dtSeconds) {
		final double busVolts = RobotController.getBatteryVoltage();
		if (motor instanceof SparkFlex flex) {
			if (flexSim == null) flexSim = new SparkFlexSim(flex, simModel.getMotor());
			simModel.update(flexSim.getAppliedOutput() * busVolts, dtSeconds);
			flexSim.iterate(simModel.getRotorRPS() * 60.0, busVolts, dtSeconds);
			if (encoderType == EncoderType.Absolute && absoluteSim == null) absoluteSim = new SparkAbsoluteEncoderSim(flex);
		} else {
			if (maxSim == null) maxSim = new SparkMaxSim((SparkMax) motor, simModel.getMotor());
			simModel.update(maxSim.getAppliedOutput() * busVolts, dtSeconds);
			maxSim.iterate(simModel.getRotorRPS() * 60.0, busVolts, dtSeconds);
			if (encoderType == EncoderType.Absolute && absoluteSim == null) absoluteSim = new SparkAbsoluteEncoderSim((SparkMax) motor);
		}

		if (absoluteSim != null) {
			// In the encoder's units after its conversion factors: rotations and RPM
			absoluteSim.setPosition(absoluteSimOffset + simModel.getRotorRotations() / motorRotationsPerSensorRotation);
			absoluteSim.setVelocity(simModel.getRotorRPS() * 60.0 / motorRotationsPerSensorRotation);
		}
	}

	private double getSimRotorRotations() {
		return simModel == null ? 0 : simModel.getRotorRotations();
	}

	@Override
	public double getTemperature() {
		return motor.getMotorTemperature();
	}

	@Override
	public void enableVoltageCompensation(double volts) {
		enableVoltageCompensationNoApply(volts);
		configure();
	}

	@Override
	public void enableVoltageCompensationNoApply(double volts) {
		config.voltageCompensation(volts);
	}

	@Override
	public void setBrakeMode() {
		setBrakeModeNoApply();
		configure();
	}

	@Override
	public void setBrakeModeNoApply() {
		config.idleMode(IdleMode.kBrake);
	}

	@Override
	public void setCoastMode() {
		setCoastModeNoApply();
		configure();
	}

	@Override
	public void setCoastModeNoApply() {
		config.idleMode(IdleMode.kCoast);
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
