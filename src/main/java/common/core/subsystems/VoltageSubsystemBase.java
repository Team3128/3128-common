package common.core.subsystems;

import java.util.Arrays;
import java.util.List;

import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

/**
 * Team 3128's Manipulator Template class.
 * @since 2024 Crescendo
 * @author Teja Yaramada
 */
public abstract class VoltageSubsystemBase extends SubsystemBase implements NAR_Subsystem {
    
    protected final List<NAR_Motor> motors;
    private double currentThreshold;

    public VoltageSubsystemBase(double currentThreshold, NAR_Motor... motors){
        requireNonNullParam(motors, "motors", "VoltageSubsystemBase");
        
        this.motors = List.of(motors);
        this.currentThreshold = currentThreshold;
        
        configMotors();
    }

    public VoltageSubsystemBase(NAR_Motor... motors){
        this(30, motors);
    }

    /**
     * Configure motor settings.
     */
    protected abstract void configMotors();
    
    /**
     * Returns whether manipulator has an object.
     */
    public boolean hasObjectPresent(){
        return Math.abs(getCurrent()) > currentThreshold;
    }
    
    /**
     * Sets power to motor.
     * 
     * @param power Setpoint the pivot goes to.
     * @return Command setting pivot setpoint.
     */
    public Command run(double power) {
        return runOnce(()-> motors.forEach(motor -> motor.set(power)));
    }


    public Command runVolts(double volts) {
        return runOnce(()-> motors.forEach(motor -> motor.setVolts(volts)));
    }

    /**
     * Stops all motors in the subsystem.
     * 
     * @return Command stopping the subsystem motors.
     */
    public Command stop(){
        return run(0);
    }

    /**
     * Returns current of the first motor.
     */
    public double getCurrent(){
        return motors.get(0).getStallCurrent();
    }

    public void setNeutralMode(Neutral mode) {
        motors.forEach(motor -> motor.setNeutralMode(mode));
    }

    public double getPosition() {
        return motors.get(0).getPosition();
    }

    public double getVelocity() {
        return motors.get(0).getVelocity();
    }
}