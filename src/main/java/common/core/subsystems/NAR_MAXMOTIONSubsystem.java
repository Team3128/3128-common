package common.core.subsystems;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.Log;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import common.core.controllers.Controller;

public class NAR_MAXMOTIONSubsystem extends NAR_PIDSubsystem implements NAR_Subsystem{
    protected SparkFlex motor;
    protected SparkClosedLoopController controller;

    private SparkFlexConfig flexConfig;

    private DoubleSupplier feedforward;

    public NAR_MAXMOTIONSubsystem(PIDFFConfig config, NAR_CANSpark m_motor, double maxVelocity, double maxAcceleration) {
        super(new Controller(config, Controller.Type.POSITION));

        this.motor = (SparkFlex) m_motor.getMotor();
        this.controller = motor.getClosedLoopController();
        
        this.flexConfig = new SparkFlexConfig();
        flexConfig.closedLoop.maxMotion.cruiseVelocity(maxVelocity);
        flexConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
        flexConfig.closedLoop.p(config.kP, ClosedLoopSlot.kSlot0);
        flexConfig.closedLoop.i(config.kI, ClosedLoopSlot.kSlot0);
        flexConfig.closedLoop.d(config.kD, ClosedLoopSlot.kSlot0);

        motor.configure(flexConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        
        feedforward = () -> {return (config.kS.getAsDouble() + config.kA.getAsDouble() + config.kV.getAsDouble() + config.kG_Function.getAsDouble());};
    }

    
    /**
     * Sets power to motors.
     * 
     * @param power The power to set the motors to.
     */
    public void run(double power) {
        motor.set(power);
    }

    /**
     * Command setting power to motors.
     * 
     * @param power The power to set the motors to.
     * @return Command setting the power to the motors.
     */
    public Command runCommand(double power) {
        return runOnce(() -> run(power));
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        Log.info(getName(), "Disabling");
        disable();
        run(0);
    }

    /**
     * Command stopping all motors.
     * 
     * @return Command stopping all the motors.
     */
    public Command stopCommand(){
        return runOnce(()-> stop());
    }

    /**
     * Resets measurement position to controller position minimum.
     */
    public void reset() {
        
    }

    /**
     * Returns the position of the motor
     */
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    /**
     * Returns the velocity of the motor
     */
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    /**
     * 
     * Uses MAXMotion to get to the setpoint
     * 
     * @param setpoint position to go to in rotations 
     */
    public void pidTo(double setpoint) {
        Log.info("maxmotion", setpoint);
        controller.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    /**
     * 
     * Uses MAXMotion to get to the setpoint
     * 
     * @param setpoint position to go to in rotations 
     */
    public void pidTo(DoubleSupplier setpoint) {
        pidTo(setpoint.getAsDouble());
    }

    /**
     * 
     * Uses MAXMotion to get to the velocity
     * 
     * @param velocity velocity to get to in rotations per second 
     */
    public Command velocityTo(double velocity) {
        return runOnce(() -> controller.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, feedforward.getAsDouble()));
    }

    /**
     * 
     * Uses MAXMotion to get to the velocity
     * 
     * @param velocity velocity to get to in rotations per second 
     */
    public Command velocityTo(DoubleSupplier velocity) {
        return velocityTo(velocity.getAsDouble());
    }


    @Override
    public Command resetCommand() {
        return runOnce(() -> reset());
    }


    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }


    @Override
    public Command runVoltsCommand(double volts) {
        return runOnce(() -> runVolts(volts));
    }


    @Override
    public void setNeutralMode(Neutral mode) {
        switch (mode) {
            case BRAKE:
                flexConfig.idleMode(IdleMode.kBrake);
                break;
            case COAST:
                flexConfig.idleMode(IdleMode.kCoast);
                break;
        }

        motor.configure(flexConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }


    @Override
    public double getVolts() {
        return motor.getBusVoltage();
    }
}