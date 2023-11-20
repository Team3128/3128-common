package common.hardware.motorcontroller;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import common.core.NAR_Robot;
import common.utility.NAR_Shuffleboard;
import static common.hardware.motorcontroller.MotorControllerConstants.*;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Team 3128's streamlined {@link CANSparkMax} class.
 * @since 2023 CHARGED UP
 * @author Mason Lam
 */
public class NAR_CANSparkMax extends NAR_Motor {
	/**
	 * Team 3128's status frames
	 */
	public enum SparkMaxConfig {
		DEFAULT(MAX_PRIORITY, HIGH_PRIORITY, HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		FOLLOWER(MEDIUM_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		POSITION(MAX_PRIORITY, MEDIUM_PRIORITY, HIGH_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		VELOCITY(MAX_PRIORITY, HIGH_PRIORITY, MEDIUM_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY),
		ABSOLUTE(MAX_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, NO_PRIORITY, HIGH_PRIORITY, HIGH_PRIORITY);

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
	
	private double kP, kI, kD;
	private EncoderType encoderType;
	private SparkMaxRelativeEncoder relativeEncoder;
	private SparkMaxAbsoluteEncoder absoluteEncoder;
	private final SparkMaxPIDController controller;
    protected final CANSparkMax motor;

    /**
	 * Create a new object to control a SPARK MAX motor
	 *
	 * @param deviceNumber The device ID.
	 * @param encoderType The type of encoder used, ie. relative build in encoder or absolute encoder
	 * 		connected by an adapter.
	 * @param type The motor type connected to the controller. Brushless motor wires must be connected
	 *     	to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *     	connected to the Red and Black terminals only.
	 * @param kP The proportional coefficient of the on board PIDController.
	 * @param kI The integral coefficient of the on board PIDController.
   	 * @param kD The derivative coefficient of the on board PIDController.
	 */
    public NAR_CANSparkMax(int deviceNumber, MotorType type, EncoderType encoderType, double kP, double kI, double kD) {
        motor = new CANSparkMax(deviceNumber, type);

        motor.setCANTimeout(0);
		motor.restoreFactoryDefaults(); // Reset config parameters, unfollow other motor controllers
		motor.enableVoltageCompensation(12);

		this.encoderType = encoderType;

		if (encoderType == EncoderType.Relative) {
			relativeEncoder = (SparkMaxRelativeEncoder) motor.getEncoder();
			relativeEncoder.setAverageDepth(2);
			relativeEncoder.setMeasurementPeriod(10);
		}
		
		else {
			absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
			absoluteEncoder.setVelocityConversionFactor(60);
			absoluteEncoder.setAverageDepth(2);
		}

		controller = motor.getPIDController();
		controller.setOutputRange(-1, 1);
		controller.setP(kP);
		controller.setI(kI);
		controller.setD(kD);
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		
		controller.setFeedbackDevice(encoderType == EncoderType.Relative ? relativeEncoder : absoluteEncoder);

		motor.burnFlash();
    }

    /**
	 * Create a new object to control a SPARK MAX motor
	 *
	 * @param deviceNumber The device ID.
     * @param type The motor type connected to the controller. Brushless motor wires must be connected
	 *     	to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *     	connected to the Red and Black terminals only.
	 * @param encoderType The type of encoder used, ie. relative build in encoder or absolute encoder
	 * 		connected by an adapter.
	 */
	public NAR_CANSparkMax(int deviceNumber, MotorType type, EncoderType encoderType) {
		this(deviceNumber, type, encoderType, 0, 0, 0);
	}

    /**
	 * Create a new object to control a SPARK MAX motor
	 *
	 * @param deviceNumber The device ID.
     * @param type The motor type connected to the controller. Brushless motor wires must be connected
	 *     	to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *     	connected to the Red and Black terminals only.
	 */
	public NAR_CANSparkMax(int deviceNumber, MotorType type) {
		this(deviceNumber, type, EncoderType.Relative);
	}
    
    /**
	 * Create a new object to control a SPARK MAX motor
	 *
	 * @param deviceNumber The device ID.
	 */
	public NAR_CANSparkMax(int deviceNumber) {
		this(deviceNumber, MotorType.kBrushless);
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
		}, 0.5);
		NAR_Robot.addPeriodic(()-> {
			if (integral.getAsDouble() == kI) {
				kI = proportional.getAsDouble();
				controller.setI(kI);
			}
		},0.5);
		NAR_Robot.addPeriodic(()-> {
			if (derivative.getAsDouble() == kD) {
				kD = derivative.getAsDouble();
				controller.setD(kD);
			}
		}, 0.5);
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
	public void setCurrentLimit(int limit) {
		motor.setSmartCurrentLimit(limit);
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
	 * @return {@link REVLibError#kOk} if successful
	 */
	public REVLibError setPeriodicFramePeriod(PeriodicFrame frame, int periodMs) {
		return motor.setPeriodicFramePeriod(frame, periodMs);
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

    /** Enables continuous input.
    *
    * <p>Rather then using the max and min input range as constraints, the motor considers them to be the
    * same point and automatically calculates the shortest route to the setpoint.
    *
    * @param minInput The minimum value expected from the input.
    * @param maxInput The maximum value expected from the input.
    */
    public void enableContinuousInput(double minInput, double maxInput) {
        enableContinuousInput(minInput, maxInput, unitConversionFactor);
    }

    /**
	 * Enables continuous input.
	 *
	 * <p>Rather then using the max and min input range as constraints, it considers them to be the
	 * same point and automatically calculates the shortest route to the setpoint.
	 *
	 * @param minInput The minimum value expected from the input.
	 * @param maxInput The maximum value expected from the input.
	 * @param factor The conversion factor to multiply the inputs by.
	 */
	public void enableContinuousInput(double minInput, double maxInput, double factor) {
		controller.setPositionPIDWrappingEnabled(true);
		controller.setPositionPIDWrappingMinInput(minInput / factor);
		controller.setPositionPIDWrappingMaxInput(maxInput / factor);
	}

	/**
	 * Burns all settings to flash; stores settings between power cycles
	 */
	public void burnFlash() {
		motor.burnFlash();
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
	public void enableVoltageCompensation(double volts) {
		motor.enableVoltageCompensation(volts);
	}

	@Override
	protected void setBrakeMode() {
		motor.setIdleMode(IdleMode.kBrake);
	}

	@Override
	protected void setCoastMode() {
		motor.setIdleMode(IdleMode.kCoast);
	}

	@Override
    public CANSparkMax getMotor() {
        return motor;
    }
}
