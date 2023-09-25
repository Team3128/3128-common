package common.hardware.motorcontroller;

import static common.hardware.motorcontroller.MotorControllerConstants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class NAR_TalonSRX extends NAR_Motor {

	protected WPI_TalonSRX motor;

	/**
	 * @param deviceNumber device id
	 */
	public NAR_TalonSRX(int deviceNumber) {
		motor = new WPI_TalonSRX(deviceNumber);
	}

	@Override
	public void enableVoltageCompensation(double volts) {
		motor.enableVoltageCompensation(true);
		motor.configVoltageCompSaturation(volts);
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
	protected void setVoltage(double volts) {
		motor.set(volts / 12.0);
	}

	@Override
	protected void setVelocity(double rpm, double feedForward) {
		motor.set(ControlMode.Velocity, rpm * RPM_TO_TALONSRX, DemandType.ArbitraryFeedForward, feedForward / 12.0);
	}

	@Override
	protected void setPosition(double rotations, double feedForward) {
		motor.set(ControlMode.Position, rotations * TALONSRX_ENCODER_RESOLUTION, DemandType.ArbitraryFeedForward, feedForward / 12.0);
	}

	public double getStatorCurrent() {
		return motor.getStatorCurrent();
	}

	@Override
	public double getAppliedOutput() {
		return motor.getMotorOutputPercent();
	}

	@Override
	public void resetRawPosition(double rotations) {
		motor.setSelectedSensorPosition(rotations * TALONSRX_ENCODER_RESOLUTION);
	}

	@Override
	public double getRawPosition() {
		return motor.getSelectedSensorPosition() / TALONSRX_ENCODER_RESOLUTION;
	}

	@Override
	public double getRawVelocity() {
		return motor.getSelectedSensorVelocity() / RPM_TO_TALONSRX;
	}

	@Override
	public WPI_TalonSRX getMotor() {
		return motor;
	}
}