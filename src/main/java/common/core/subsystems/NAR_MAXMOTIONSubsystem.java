package common.core.subsystems;

import common.core.controllers.ControllerBase;
import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.utility.Log;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import common.core.controllers.Controller;

public class NAR_MAXMOTIONSubsystem extends NAR_PIDSubsystem {
    protected SparkFlex motor;
    private RelativeEncoder m_encoder;
    protected SparkClosedLoopController controller;

    private SparkFlexConfig flexConfig;

    private DoubleSupplier feedforward;

    public NAR_MAXMOTIONSubsystem(PIDFFConfig config, NAR_CANSpark m_motor, double maxVelocity, double maxAcceleration) {
        super(new Controller(config, Controller.Type.POSITION));

        this.motor = (SparkFlex) m_motor.getMotor();
        this.m_encoder = motor.getEncoder();
        this.controller = motor.getClosedLoopController();
        
        this.flexConfig = new SparkFlexConfig();
        flexConfig.closedLoop.maxMotion.maxVelocity(maxVelocity);
        flexConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
        flexConfig.closedLoop.p(config.kP, ClosedLoopSlot.kSlot0);
        flexConfig.closedLoop.i(config.kI, ClosedLoopSlot.kSlot0);
        flexConfig.closedLoop.d(config.kD, ClosedLoopSlot.kSlot0);

        motor.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
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
        m_encoder.setPosition(0);
    }

    /**
     * Get the position of the mechanism relative to its reset.
     * 
     * @return The position of the first motor.
     */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Get the velocity of the mechanism.
     * 
     * @return The velocity of the first motor.
     */
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    /**
     * 
     * Uses MAXMotion to get to the setpoint
     * 
     * @param setpoint position to go to in rotations 
     */
    public Command pidTo(double setpoint) {
        return runOnce(() -> controller.setReference(setpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedforward.getAsDouble()));
    }

    /**
     * 
     * Uses MAXMotion to get to the setpoint
     * 
     * @param setpoint position to go to in rotations 
     */
    public Command pidTo(DoubleSupplier setpoint) {
        return pidTo(setpoint.getAsDouble());
    }

    /**
     * 
     * Uses MAXMotion to get to the velocity
     * 
     * @param velocity velocity to get to in rotations per second 
     */
    public Command velocityTo(double velocity) {
        return runOnce(() -> controller.setReference(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, feedforward.getAsDouble()));
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
}