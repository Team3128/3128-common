package common.core.subsystems;

import java.util.Arrays;

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
    
    protected final NAR_Motor leader;
    private double currentThreshold;

    public VoltageSubsystemBase(double currentThreshold, NAR_Motor leader, NAR_Motor... followers){
        requireNonNullParam(leader, "leader", "PositionSubsystemBase");
        
        this.leader = leader;
        Arrays.stream(followers).forEach((follower)-> follower.follow(this.leader));
        this.currentThreshold = currentThreshold;
        
        configMotors();
    }

    public VoltageSubsystemBase(NAR_Motor leader, NAR_Motor... followers){
        this(30, leader, followers);
    }

    /**
     * Configure motor settings.
     */
    protected abstract void configMotors();

    
    /**
     * Returns current of the first motor.
     */
    public double getCurrent(){
        return leader.getStallCurrent();
    }
    
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
        return runOnce(()-> leader.set(power));
    }


    public Command runVolts(double volts) {
        return runOnce(()-> leader.setVolts(volts));
    }

    /**
     * Stops all motors in the subsystem.
     * 
     * @return Command stopping the subsystem motors.
     */
    public Command stop(){
        return run(0);
    }

    public void setNeutralMode(Neutral mode) {
        leader.setNeutralMode(mode);
    }
}