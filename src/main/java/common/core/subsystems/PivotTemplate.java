package common.core.subsystems;

import common.core.controllers.ControllerBase;
import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Team 3128's Pivot Template class.
 * @since 2024 Crescendo
 * @author Maggie Liang
 */
public abstract class PivotTemplate extends NAR_PIDSubsystem{

    protected final NAR_Motor[] motors;
    
    /**
     * Creates an Pivot object.
     * @param controller Controller to control motor output.
     * @param motors Pivot motors.
     */
    public PivotTemplate(ControllerBase controller, NAR_Motor...motors){
        super(controller);
        this.motors = motors;
        configMotors();
    }
    
    /**
     * Configure motor settings
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
    public void setPower(double power){
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
     * Moves pivot to a setpoint.
     * @param setpoint Setpoint the pivot goes to.
     * @return Command setting pivot setpoint.
     */
    public Command pivotTo(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }
    
    /**
     * Sets pivot power.
     * @param power Power the motor is run at.
     * @return Command setting pivot power.
     */
    public Command runPivot(double power){
        return runOnce(() -> setPower(power));
    }
    
    /**
     * Reset pivot position.
     * @param position Position to reset to.
     * @return Command that resets the pivot position.
     */
    public Command reset(double position) {
        return runOnce(()-> motors[0].resetPosition(position));
    }
}
