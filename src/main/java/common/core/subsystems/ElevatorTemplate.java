package common.core.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import common.core.controllers.ControllerBase;
import common.hardware.motorcontroller.NAR_Motor;

/**
 * Team 3128's Elevator Template class.
 * @since 2024 Crescendo
 * @author Maggie Liang
 */
public abstract class ElevatorTemplate extends NAR_PIDSubsystem {
    
    protected final NAR_Motor[] motors;
    
    /**
     * Creates an Elevator object.
     * @param controller Controller to control motor output.
     * @param motors Elevator motors.
     */
    public ElevatorTemplate(ControllerBase controller, NAR_Motor... motors) {
        super(controller);
        this.motors = motors;
        configMotors();
    }
    
    /**
     * Configures motor settings.
     */
    protected abstract void configMotors();
    
    @Override
    protected void useOutput(double output, double setpoint) {
        for (final NAR_Motor motor : motors){
            motor.setVolts(output);
        }
    }
    
    /**
     * Sets power to all motors.
     *
     * @param power The power of the motor on [-1, 1]
     */
    private void setPower(double power) {
        disable();
        for (final NAR_Motor motor : motors){
            motor.set(power);
        }
    }

    @Override
    public double getMeasurement() {
        return motors[0].getPosition();
    }
    
    /**
     * Moves elevator to a setpoint.
     * @param setpoint Setpoint the elevator goes to.
     * @return Command setting elevator setpoint.
     */
    public Command moveElevator(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }
    
    /**
     * Sets elevator power.
     * @param power Power the motor is run at.
     * @return Command setting elevator power.
     */
    public Command runElevator(double power){
        return runOnce(() -> setPower(power));
    }
    
    /**
     * Reset elevator position.
     * @param position Position to reset to.
     * @return Command that resets the elevator position.
     */
    public Command reset(double position) {
        return runOnce(()-> {
            
        for(NAR_Motor motor : motors){
            motor.resetPosition(position);
        }
    });
    }
}
