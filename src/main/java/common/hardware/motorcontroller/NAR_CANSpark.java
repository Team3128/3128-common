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

import java.util.function.Supplier;

import static common.hardware.motorcontroller.MotorControllerConstants.*;

/**
 * A REV Spark MAX or Spark Flex motor controller, which drives NEO, NEO 550 and NEO Vortex
 * motors, using REVLib.
 *
 * <p>Every setting is kept in one {@code SparkBaseConfig}. Changing a setting updates it and sends
 * it to the Spark without saving it; {@code NAR_Robot} saves every Spark's settings to flash once
 * after startup, as REV recommends (see {@link #burnFlash()}).
 *
 * <p>The Spark's own control loops measure in rotations and rotations per minute, output a
 * fraction of 12 V, and run every millisecond. {@link NAR_Motor} converts gains and setpoints to
 * and from your units.
 *
 * <p>It can read either the motor's built-in encoder or an absolute encoder plugged into the
 * Spark; see {@link EncoderType}.
 *
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public class NAR_CANSpark extends NAR_Motor {

	private static int numFailedConfigs = 0;

	/** How many times to try sending settings before giving up. */
	public static int maximumRetries = 5;

	/** Every Spark created, so they can all be saved to flash at once. */
	public static final LinkedList<NAR_CANSpark> instances = new LinkedList<NAR_CANSpark>();

	/**
	 * Saves every Spark's settings to flash; see {@link #burnFlash()}. {@code NAR_Robot} calls
	 * this once after {@code robotInit}.
	 */
	public static void burnFlashAll() {
		for (final NAR_CANSpark spark : instances) {
			spark.burnFlash();
		}
	}

	/**
	 * @return How many settings, across every Spark, never got through after all the retries.
	 */
	public static int getNumFailedConfigs() {
		return numFailedConfigs;
	}

	/**
	 * How often each of the Spark's status frames is sent, in milliseconds:
	 * <ul>
	 * <li>0: output and faults</li>
	 * <li>1: velocity, current, temperature and voltage</li>
	 * <li>2: position</li>
	 * <li>3: an analog sensor</li>
	 * <li>4: an alternate encoder</li>
	 * <li>5 and 6: an absolute encoder's position and velocity</li>
	 * </ul>
	 * Frame 1 is never turned off, since every mechanism logs its motors' current and temperature.
	 */
	public enum NAR_SparkMaxConfig {
		DEFAULT(HIGH_PRIORITY, HIGH_PRIORITY, HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		FOLLOWER(HIGH_PRIORITY, MEDIUM_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		POSITION(HIGH_PRIORITY, MEDIUM_PRIORITY, HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		VELOCITY(HIGH_PRIORITY, HIGH_PRIORITY, LOW_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		ABSOLUTE(HIGH_PRIORITY, MEDIUM_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, HIGH_PRIORITY, HIGH_PRIORITY);

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
	 * Which encoder the Spark reads.
	 */
    public enum EncoderType {
		/** The motor's built-in encoder. It starts at 0 every time the robot turns on. */
		Relative,
		/**
		 * An absolute encoder, like a REV Through Bore, plugged into the Spark. It knows where it
		 * is as soon as the robot turns on, from 0 to 1 rotation.
		 */
		Absolute
	}

	/**
	 * Which Spark it is.
	 */
	public enum ControllerType {
		/** A Spark MAX, usually driving a NEO or NEO 550. */
		CAN_SPARK_MAX,
		/** A Spark Flex, usually driving a NEO Vortex. */
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
		// Safe settings until a mechanism's MotorConfig replaces them
		enableVoltageCompensationNoApply(12.0);
		setStatorLimitNoApply(motorType == MotorType.kBrushless ? NEO_STATOR_CurrentLimit : NEO_STATOR_550CurrentLimit);
		setSupplyLimitNoApply(motorType == MotorType.kBrushless ? NEO_SUPPLY_CurrentLimit : NEO_SUPPLY_550CurrentLimit);

		this.encoderType = encoderType;

		relativeEncoder = motor.getEncoder();
		if (encoderType == EncoderType.Relative) {
			// The Spark works out speed by averaging encoder readings. REV's defaults average over a
			// long time, so the speed it reports lags tens of milliseconds behind the real one;
			// these, from Mechanical Advantage, cut the lag
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

	/**
	 * Sends the settings to the Spark without saving them, as REV recommends for changes after
	 * startup: saving is slow and wears out its flash. {@link #burnFlash()} saves them once at startup.
	 */
	public void configure() {
		configSpark(() -> motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
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
	 * Sets REV's "smart current limit": the Spark lowers its output to keep the motor's current
	 * under this. NEOs draw huge spikes without one, which can damage the motor and the Spark, so
	 * always set it.
	 *
	 * @param limit Amps
	 */
	@Override
	public void setStatorLimit(int limit) {
		setStatorLimitNoApply(limit);
		configure();
	}

	/** Stores the smart current limit, without sending it yet. */
	@Override
	public void setStatorLimitNoApply(int limit) {
		config.smartCurrentLimit(limit);
	}

	/**
	 * Sets REV's "secondary current limit". Sparks don't have a true battery-side (supply) limit
	 * like a TalonFX, so this is the closest thing: if the motor's current goes over it, the Spark
	 * cuts its output for a moment. Keep it above {@link #setStatorLimit(int)}, which does the
	 * everyday limiting.
	 *
	 * @param limit Amps
	 */
	@Override
	public void setSupplyLimit(int limit) {
		setSupplyLimitNoApply(limit);
		configure();
	}

	/** Stores the secondary current limit, without sending it yet. */
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

	/**
	 * Stores a frame's rate, without sending it yet. REVLib sets rates per signal rather than per
	 * frame, so this sets every signal the frame carries. Frame 4, an alternate encoder, isn't
	 * supported and is ignored.
	 */
	public void setPeriodicFramePeriodNoApply(PeriodicFrame frame, int periodMs) {
		switch (frame) {
			case kStatus0:
				config.signals.appliedOutputPeriodMs(periodMs).faultsPeriodMs(periodMs);
				break;
			case kStatus1:
				config.signals.primaryEncoderVelocityPeriodMs(periodMs).motorTemperaturePeriodMs(periodMs)
					.busVoltagePeriodMs(periodMs).outputCurrentPeriodMs(periodMs);
				break;
			case kStatus2:
				config.signals.primaryEncoderPositionPeriodMs(periodMs);
				break;
			case kStatus3:
				config.signals.analogVoltagePeriodMs(periodMs).analogVelocityPeriodMs(periodMs).analogPositionPeriodMs(periodMs);
				break;
			case kStatus5:
				config.signals.absoluteEncoderPositionPeriodMs(periodMs);
				break;
			case kStatus6:
				config.signals.absoluteEncoderVelocityPeriodMs(periodMs);
				break;
			default:
				break;
		}
	}

	/**
	 * Sets every frame's rate to one of the team's presets in {@link NAR_SparkMaxConfig}.
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
	 * Resets the Spark to its defaults, applies every setting, and saves them to flash, so it keeps
	 * them if it reboots, e.g. after a brownout. REV recommends doing this once at startup;
	 * {@code NAR_Robot} does it for every Spark after {@code robotInit}.
	 */
	public REVLibError burnFlash() {
		if (RobotBase.isSimulation()) return REVLibError.kOk;

		// configure() waits for the Spark to confirm, so there's no need to pause around it
		REVLibError status = REVLibError.kOk;
		for (int i = 0; i < maximumRetries; i++) {
			status = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
			if (status == REVLibError.kOk) break;
		}
		if (status == REVLibError.kOk) {
			Log.info("Motors", "Burned flash for Spark Max " + motor.getDeviceId());
		} else {
			Log.unusual("Motors", "Failed to burn flash for Spark Max " + motor.getDeviceId());
		}

		return status;
	}

	/**
     * Makes this Spark copy another Spark's output on the Sparks themselves, so it follows
     * instantly. Its own inversion setting is ignored while following.
     * @param leader The Spark to copy
	 * @param invert Whether to spin the opposite way to the leader
     */
	public void follow(NAR_CANSpark leader, boolean invert) {
		followNoApply(leader, invert);
		configure();
	}

	/** Stores the follow setting, without sending it yet. */
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

	/** REVLib's {@code SparkMax} or {@code SparkFlex}, for anything this class doesn't cover. */
	@Override
    public SparkBase getMotor() {
        return motor;
    }

	/** Every fault the Spark is reporting right now, like over-temperature or a sensor problem. */
	public Faults getAllFaults() {
		return motor.getFaults();
	}

	//TODO: Eval necessity
	// public boolean getFault(int fault) {
	// 	return motor.getFault(fault);
	// }

	/**
	 * RUNNING if the Spark has no faults, now or since it last cleared them, and its last command
	 * worked; DISCONNECTED otherwise.
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
        disableSim();
        motor.close();
    }
}
