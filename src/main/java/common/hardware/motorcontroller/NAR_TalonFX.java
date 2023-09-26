package common.hardware.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import common.core.NAR_Robot;
import common.utility.NAR_Shuffleboard;

import static common.hardware.motorcontroller.MotorControllerConstants.*;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Team 3128's streamlined {@link WPI_TalonFX} class.
 * @since 2023 CHARGED UP
 * @author Mason Lam
 */
public class NAR_TalonFX extends NAR_Motor{

    protected final WPI_TalonFX motor;
    private double kP, kI, kD;

    /**
	 * Create a new object to control a Falcon 500 motor
	 *
	 * @param deviceNumber The device ID.
	 * @param canBus name of the CanBus, change when using Canivore
	 * @param kP The proportional coefficient of the on board PIDController.
	 * @param kI The integral coefficient of the on board PIDController.
   	 * @param kD The derivative coefficient of the on board PIDController.
	 */
    public NAR_TalonFX(int deviceNumber, String canBus, double kP, double kI, double kD) {
        motor = new WPI_TalonFX(deviceNumber, canBus);

        enableVoltageCompensation(12);
		motor.configFactoryDefault();

        this.kP = kP;
		this.kI = kI;
		this.kD = kD;

		motor.config_kP(0, kP);
		motor.config_kI(0, kI);
		motor.config_kD(0, kD);
    }

    /**
	 * Create a new object to control a Falcon 500 motor
	 *
	 * @param deviceNumber The device ID.
	 * @param canBus name of the CanBus, change when using Canivore
	 */
	public NAR_TalonFX(int deviceNumber, String canBus) {
		this(deviceNumber, canBus, 0, 0, 0);
	}

	/**
	 * Create a new object to control a Falcon 500 motor
	 *
	 * @param deviceNumber The device ID.
	 */
	public NAR_TalonFX(int deviceNumber) {
		this(deviceNumber, "");
	}

    /**
	 * Adds a PID tuning setup to a specific shuffleboard tab. Editing a value on the tab 
	 * will automatically update the value on the controller.
	 * @param tabName The title of the tab to select.
	 * @param prefix String added before each name of PID widgets.
	 * @param column Column to add the PID widgets to.
	 */
	public void initShuffleboard(String tabName, String prefix, int column, Consumer<Runnable> addPeriodic) {
		DoubleSupplier proportional = NAR_Shuffleboard.debug(tabName, prefix + "_kP", kP, column, 0);
		DoubleSupplier integral = NAR_Shuffleboard.debug(tabName, prefix + "_kI", kI, column, 1);
		DoubleSupplier derivative = NAR_Shuffleboard.debug(tabName, prefix + "_kD", kD, column, 2);
		NAR_Robot.addPeriodic(()-> {
			if (proportional.getAsDouble() == kP) {
				kP = proportional.getAsDouble();
				motor.config_kP(0, kP);
			}
		}, 0.5);
		NAR_Robot.addPeriodic(()-> {
			if (integral.getAsDouble() == kI) {
				kI = proportional.getAsDouble();
				motor.config_kI(0, kI);
			}
		}, 0.5);
		NAR_Robot.addPeriodic(()-> {
			if (derivative.getAsDouble() == kD) {
				kD = derivative.getAsDouble();
				motor.config_kD(0, kD);
			}
		}, 0.5);
	}

    /**
	 * Set the rate of transmission for status frames from the TalonFX
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to change the default rates.
	 * 
	 * <p><b>Status 1</b>: Applied Output, Faults information, Limit Switch Information
	 * <p><b>Status 2</b>: Selected Sensor Position, Selected Sensor Velocity, Brushed Current Measurement, Sticky Fault information
	 * <p><b>Status 3</b>: Quadrature information
	 * <p><b>Status 4</b>: Analog Input, Supply Battery Voltage, Controller Temperature
	 * <p><b>Status 8</b>: Pulse Width information
	 * <p><b>Status 10</b>: Motion profiling/Motion magic information
	 * <p><b>Status 12</b>: Selected Sensor Position (AUX PID), Selected Sensor Velocity (AUX PID)
	 * <p><b>Status 13</b>: PID0 Primary PID information
	 * <p><b>Status 14</b>: PID1 Auxillary PID information
	 * <p><b>Status 21</b>: Integrated Sensor Position (TalonFX), Integrated Sensor Velocity (TalonFX)
	 * 
	 * @param frame which {@link StatusFrameEnhanced} to be changed.
	 * @param periodMs Period in ms for the given frame.
	 */
	public void setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs) {
		motor.setStatusFramePeriod(frame, periodMs);
	}

    /**
	 * Set the rate of transmission for stauts frames from the TalonFX
	 *
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to set to team 3128's default rates.
	 *
	 * <p>Defaults: Status1 - 10ms Status2 - 20ms Status3 - 255ms Status4 - 255ms Status8 - 255ms Status10
	 * - 255ms Status12 - 255ms  Status13 - 255ms  Status14 - 255ms  Status21 - 255ms
	 */
	public void setDefaultStatusFrames() {
		setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MotorControllerConstants.MAX_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MotorControllerConstants.HIGH_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, MotorControllerConstants.LOW_PRIORITY);
	}

	public void configAllSettings(TalonFXConfiguration config) {
		motor.configAllSettings(config);
	}

    @Override
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    protected void setPercentOutput(double speed) {
        motor.set(speed);
    }

    @Override
    protected void setVoltage(double volts) {
        motor.set(volts / 12.0);
    }

    @Override
    protected void setVelocity(double rpm, double feedForward) {
        motor.set(ControlMode.Velocity, rpm * RPM_TO_FALCON, DemandType.ArbitraryFeedForward, feedForward / 12.0);
    }

    @Override
    protected void setPosition(double rotations, double feedForward) {
        motor.set(ControlMode.Position, rotations * FALCON_ENCODER_RESOLUTION, DemandType.ArbitraryFeedForward, feedForward / 12.0);
    }

    @Override
    public double getAppliedOutput() {
        return motor.getMotorOutputPercent();
    }

	@Override
	protected void resetRawPosition(double rotations) {
		motor.setSelectedSensorPosition(rotations * FALCON_ENCODER_RESOLUTION);
	}

    @Override
    protected double getRawPosition() {
        return motor.getSelectedSensorPosition() / MotorControllerConstants.FALCON_ENCODER_RESOLUTION;
    }

    @Override
    protected double getRawVelocity() {
        return motor.getSelectedSensorVelocity() / MotorControllerConstants.RPM_TO_FALCON;
    }

	@Override
	public void enableVoltageCompensation(double volts) {
		motor.configVoltageCompSaturation(volts);
		motor.enableVoltageCompensation(true);
	}

	@Override
	protected void setBrakeMode() {
		motor.setNeutralMode(NeutralMode.Brake);
	}

	@Override
	protected void setCoastMode() {
		motor.setNeutralMode(NeutralMode.Coast);
	}
    
    @Override
    public WPI_TalonFX getMotor() {
        return motor;
    }
}
