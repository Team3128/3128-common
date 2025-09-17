package common.core.subsystems;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import common.core.controllers.Controller;
import common.core.controllers.ControllerBase;
import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands.*;

public class NAR_TalonPIDSubsystem extends NAR_PIDSubsystem {
    enum Type {
        POSITION,
        VELOCITY;
    }
    
    protected NAR_TalonFX kraken;
    private TalonFXConfiguration talonFXConfigs;
    private Slot0Configs slot0Configs;

    protected double maxVelocity;
    protected double maxAcceleration;


    protected Type type;
    protected PIDFFConfig config;
    protected TalonFX motor;

    /**
     * Creates a base controller object to control motion.
     * <p>Sets kP, kI, kD, kS, kV, kA, kG, constraints, period values.
     * @param config PIDFFConfig object containing PID and Feedforward constants.
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public NAR_TalonPIDSubsystem(PIDFFConfig config, NAR_TalonFX motor, double maxVelocity, double maxAcceleration) {
        super(new Controller(config, Controller.Type.POSITION));
        this.talonFXConfigs = new TalonFXConfiguration();

        this.slot0Configs = this.talonFXConfigs.Slot0;

        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        this.type = Type.POSITION;

        this.config = config;

        this.motor = motor.getMotor();

        configMotors();
    }

    private void configMotors() {
        slot0Configs.kP = config.kP;
        slot0Configs.kI = config.kI;
        slot0Configs.kD = config.kD;
        slot0Configs.kS = config.getkS();
        slot0Configs.kV = config.getkV();
        slot0Configs.kA = config.getkA();

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = maxAcceleration;

        motor.getConfigurator().apply(talonFXConfigs);
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
        motor.setPosition(0);
    }

    /**
     * Get the position of the mechanism relative to its reset.
     * 
     * @return The position of the first motor.
     */
    public double getPosition() {
        return motor.getRotorPosition().getValueAsDouble();
    }

    /**
     * Get the velocity of the mechanism.
     * 
     * @return The velocity of the first motor.
     */
    public double getVelocity() {
        return motor.getRotorVelocity().getValueAsDouble();
    }
}
