package common.core.subsystems;

import common.core.controllers.ControllerBase;
import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Team 3128's Shooter Template class.
 * @since 2024 Crescendo
 * @author Maggie Liang
 */
public abstract class ShooterTemplate extends NAR_PIDSubsystem {
    
    protected final NAR_Motor[] motors;
    
    /**
     * Creates an Shooter object.
     * @param controller Controller to control motor output.
     * @param motors Shooter motors.
     */
    public ShooterTemplate(ControllerBase controller, NAR_Motor... motors){
        super(controller);
        this.motors = motors;
        configMotors();
        initShuffleboard();
    }
    
    /**
     * Configures motor settings.
     */
    protected abstract void configMotors();
    
    @Override
    protected void useOutput(double output, double setpoint) {
        for (final NAR_Motor motor : motors) {
            motor.setVolts(output);
        }
    }
    
    /**
     * Sets power to all motors.
     *
     * @param power The power of the motor on [-1, 1]
     */
    protected void setPower(double power){
        disable();
        for (final NAR_Motor motor : motors) {
            motor.set(power);
        }
    }
    
    /**
     * Runs shooter at desired RPM.
     * @param setpoint Desired setpoint in RPM.
     * @return Command to run shooter at a desired setpoint.
     */
    public Command shoot(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }
    
    /**
     * Runs the shooter at a specific power.
     * @param power Power for the motors to be run at.
     * @return Command to run the shooter at a specific power.
     */
    public Command runShooter(double power){
        return runOnce(() -> setPower(power));
    }

    @Override
    public double getMeasurement() {
        return motors[0].getVelocity();
    }
    

}
