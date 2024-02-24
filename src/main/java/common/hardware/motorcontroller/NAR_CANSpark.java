package common.hardware.motorcontroller;

import java.util.LinkedList;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.FaultID;

import common.core.controllers.PIDFFConfig;
import common.core.misc.NAR_Robot;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import static common.hardware.motorcontroller.MotorControllerConstants.*;

/**
 * Team 3128's streamlined {@link CANSparkBase} class.
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
	public enum SparkMaxConfig {
		DEFAULT(MAX_PRIORITY, HIGH_PRIORITY, HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		FOLLOWER(MEDIUM_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		POSITION(LOW_PRIORITY, MEDIUM_PRIORITY, MAX_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		VELOCITY(LOW_PRIORITY, MAX_PRIORITY, MEDIUM_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		ABSOLUTE(MEDIUM_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, HIGH_PRIORITY, HIGH_PRIORITY);

		public int status0, status1, status2, status3, status4, status5, status6;

		private SparkMaxConfig(int status0, int status1, int status2, int status3, int status4, int status5, int status6) {
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
	private SparkRelativeEncoder relativeEncoder;
	private SparkAbsoluteEncoder absoluteEncoder;
	private final SparkPIDController controller;
    protected final CANSparkBase motor;

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
        motor = controllerType == ControllerType.CAN_SPARK_MAX ? new CANSparkMax(deviceNumber, motorType) : new CANSparkFlex(deviceNumber, motorType);

		configSpark(()-> motor.clearFaults());
		configSpark(()-> motor.restoreFactoryDefaults()); // Reset config parameters, unfollow other motor controllers
		configSpark(()-> motor.setCANTimeout(canSparkMaxTimeout));		//I have this here and I don't know why - Mason
		enableVoltageCompensation(12.0);
		setCurrentLimit(motorType == MotorType.kBrushless ? NEO_CurrentLimit : NEO_550CurrentLimit);

		this.encoderType = encoderType;

		if (encoderType == EncoderType.Relative) {
			relativeEncoder = (SparkRelativeEncoder) motor.getEncoder();
			//No clue what this does, but Mechanical Advantage does this so it must be good
			configSpark(()-> relativeEncoder.setAverageDepth(2));
			configSpark(()-> relativeEncoder.setMeasurementPeriod(10));
		}
		
		else {
			absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
			configSpark(()-> absoluteEncoder.setVelocityConversionFactor(60));
			configSpark(()-> absoluteEncoder.setAverageDepth(2));
		}

		controller = motor.getPIDController();
		configSpark(()-> controller.setOutputRange(-1, 1));
		configPID(PIDconfig);
		
		configSpark(()-> controller.setFeedbackDevice(encoderType == EncoderType.Relative ? relativeEncoder : absoluteEncoder));
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
		Log.fatal("Motors", "Failed to configure Spark Max " + motor.getDeviceId());
	}

	/**
	 * Adds a PID tuning setup to a specific shuffleboard tab. Editing a value on the tab 
	 * will automatically update the value on the controller.
	 * @param tabName The title of the tab to select.
	 * @param prefix String added before each name of PID widgets.
	 * @param column Column to add the PID widgets to.
	 */
	public void initShuffleboard(String tabName, String prefix, int column) {
		DoubleSupplier proportional = NAR_Shuffleboard.debug(tabName, prefix + "_kP", kP, column, 0);
		DoubleSupplier integral = NAR_Shuffleboard.debug(tabName, prefix + "_kI", kI, column, 1);
		DoubleSupplier derivative = NAR_Shuffleboard.debug(tabName, prefix + "_kD", kD, column, 2);
		NAR_Robot.addPeriodic(()-> {
			if (proportional.getAsDouble() == kP) {
				kP = proportional.getAsDouble();
				controller.setP(kP);
			}
		}, 0.5, 0.05);
		NAR_Robot.addPeriodic(()-> {
			if (integral.getAsDouble() == kI) {
				kI = proportional.getAsDouble();
				controller.setI(kI);
			}
		},0.5, 0.05);
		NAR_Robot.addPeriodic(()-> {
			if (derivative.getAsDouble() == kD) {
				kD = derivative.getAsDouble();
				controller.setD(kD);
			}
		}, 0.5, 0.05);
	}

	/**
	 * Set the PID values for the controller.
	 * @param config PIDFFConfig containing kP, kI, and kD values.
	 */
	public void configPID(PIDFFConfig config) {
		configSpark(()-> controller.setP(config.kP));
		configSpark(()-> controller.setI(config.kI));
		configSpark(()-> controller.setD(config.kD));
		this.kP = config.kP;
		this.kI = config.kI;
		this.kD = config.kD;
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
	public void setCurrentLimit(int limit) {
		configSpark(()-> motor.setSmartCurrentLimit(limit));
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
		configSpark(()-> motor.setPeriodicFramePeriod(frame, periodMs));
	}

	/**
	 * Set the rate of transmission for periodic frames from the SPARK MAX
	 *
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to set to team 3128's preset rates.
	 */
	public void setStatusFrames(SparkMaxConfig config) {
		setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.status0);
		setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.status1);
		setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.status2);
		setPeriodicFramePeriod(PeriodicFrame.kStatus3, config.status3);
		setPeriodicFramePeriod(PeriodicFrame.kStatus4, config.status4);
		setPeriodicFramePeriod(PeriodicFrame.kStatus5, config.status5);
		setPeriodicFramePeriod(PeriodicFrame.kStatus6, config.status6);
	}

	@Override
	public void setDefaultStatusFrames() {
		setStatusFrames(SparkMaxConfig.DEFAULT);
	}

	/**
	 * Burns all settings to flash; stores settings between power cycles
	 */
	public REVLibError burnFlash() {
		 if (RobotBase.isSimulation()) return REVLibError.kOk;

		Timer.delay(0.5);
		REVLibError status = motor.burnFlash();
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
		configSpark(()-> motor.follow(leader.getMotor(), invert));
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
		motor.setInverted(inverted);
	}

    @Override
    protected void setPercentOutput(double speed) {
        controller.setReference(speed, ControlType.kDutyCycle);
    }

    @Override
    protected void setVelocity(double rpm, double feedForward) {
        controller.setReference(rpm, ControlType.kVelocity, 0, feedForward);
    }

    @Override
    protected void setPosition(double rotations, double feedForward) {
        controller.setReference(rotations, ControlType.kPosition, 0, feedForward);
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
	protected void resetRawPosition(double rotations) {
		if (encoderType == EncoderType.Relative) relativeEncoder.setPosition(rotations);
	}

    @Override
    protected double getRawPosition() {
        return encoderType == EncoderType.Relative ? relativeEncoder.getPosition() : absoluteEncoder.getPosition();
    }

    @Override
    protected double getRawVelocity() {
        return encoderType == EncoderType.Relative ? relativeEncoder.getVelocity() : absoluteEncoder.getVelocity();
    }

	@Override
	public double getTemperature() {
		return motor.getMotorTemperature();
	}

	@Override
	public void enableVoltageCompensation(double volts) {
		configSpark(()-> motor.enableVoltageCompensation(volts));
	}

	@Override
	protected void setBrakeMode() {
		configSpark(()-> motor.setIdleMode(IdleMode.kBrake));
	}

	@Override
	protected void setCoastMode() {
		configSpark(()-> motor.setIdleMode(IdleMode.kCoast));
	}

	@Override
    public CANSparkBase getMotor() {
        return motor;
    }

	public short getAllFaults() {
		return motor.getFaults();
	}

	public boolean getFault(FaultID fault) {
		return motor.getFault(fault);
	}

	/**
	 * Returns motor and motor controller functionality.
	 * @return State of the motor controller and motor.
	 */
	public State getState() {
		//To do add check for motors
		if (motor.getFault(FaultID.kMotorFault) == false) {
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
