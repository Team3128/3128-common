package common.hardware.motorcontroller;

import static common.hardware.motorcontroller.MotorControllerConstants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import common.core.controllers.PIDFFConfig;
import common.utility.narwhaldashboard.NarwhalDashboard;

/**
 * Team 3128's streamlined {@link WPI_TalonSRX} class.
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public class NAR_TalonSRX extends NAR_Motor {

	protected WPI_TalonSRX motor;

	/**
	 * @param deviceNumber device id
	 */
	public NAR_TalonSRX(int deviceNumber) {
		super(deviceNumber);
		motor = new WPI_TalonSRX(deviceNumber);
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
	 * Set the rate of transmission for status frames from the TalonFX
	 *
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to set to team 3128's default rates.
	 *
	 * <p>Defaults: Status1 - 10ms Status2 - 20ms Status3 - 255ms Status4 - 255ms Status8 - 255ms Status10
	 * - 255ms Status12 - 255ms  Status13 - 255ms  Status14 - 255ms  Status21 - 255ms
	 */
	@Override
	public void setDefaultStatusFrames() {
		setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MotorControllerConstants.HIGH_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, MotorControllerConstants.LOW_PRIORITY);
	}

	@Override
	public void setPositionStatusFrames() {
        setDefaultStatusFrames();
	}

	@Override
	public void setVelocityStatusFrames() {
        setDefaultStatusFrames();
	}

	@Override
	public void setFollowerStatusFrames() {
		setDefaultStatusFrames();
	}

	/**
	 * Configures the motor settings
	 * @param config See the {@link TalonSRXConfiguration} class
	 */
	public void configAllSettings(TalonSRXConfiguration config) {
		motor.configAllSettings(config);
	}

	@Override
    public void configPID(PIDFFConfig config) {
        throw new UnsupportedOperationException("Don't use PID on a 775pro");
    }

	/**
     * Sets a motor's output based on the leader's
     * @param leader The motor to follow
     */
	public void follow(IMotorController leader) {
		motor.follow(leader);
	}

	@Override
	public void follow(NAR_Motor leader) {
		if (leader instanceof NAR_TalonSRX) {
			final NAR_TalonSRX brushedLeader = (NAR_TalonSRX) leader;
			follow(brushedLeader.getMotor());
		}
		else {
			super.follow(leader);
		}
	}

	@Override
	public void enableVoltageCompensation(double volts) {
		motor.enableVoltageCompensation(true);
		motor.configVoltageCompSaturation(volts);
	}

	/**
	 * Still burn our motors nevertheless
	 */
	@Override
	public void setCurrentLimit(int limit) {
		throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimit'");
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
	public void setInverted(boolean inverted) {
		motor.setInverted(inverted);
	}

	@Override
	protected void setPercentOutput(double speed) {
		motor.set(speed);
	}

	@Override
	protected void setVelocity(double rpm, double feedForward) {
		motor.set(ControlMode.Velocity, rpm * RPM_TO_TALONSRX, DemandType.ArbitraryFeedForward, feedForward / 12.0);
	}

	@Override
	protected void setPosition(double rotations, double feedForward) {
		motor.set(ControlMode.Position, rotations * TALONSRX_ENCODER_RESOLUTION, DemandType.ArbitraryFeedForward, feedForward / 12.0);
	}

	@Override
	public double getAppliedOutput() {
		return motor.getMotorOutputPercent();
	}

	@Override
	public double getStallCurrent() {
		return motor.getStatorCurrent();
	}

	@Override
	protected void resetRawPosition(double rotations) {
		motor.setSelectedSensorPosition(rotations * TALONSRX_ENCODER_RESOLUTION);
	}

	@Override
	protected double getRawPosition() {
		return motor.getSelectedSensorPosition() / TALONSRX_ENCODER_RESOLUTION;
	}

	@Override
	protected double getRawVelocity() {
		return motor.getSelectedSensorVelocity() / RPM_TO_TALONSRX;
	}

	@Override
	public NarwhalDashboard.State getState() {
		return NarwhalDashboard.State.RUNNING;
	}

	@Override
	public double getTemperature() {
		return motor.getTemperature();
	}

	@Override
	public WPI_TalonSRX getMotor() {
		return motor;
	}

	/**
     * Closes the talonSRX motor.
     */
    @Override
    public void close() {
        motor.close();
    }
}