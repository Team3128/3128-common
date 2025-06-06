package common.core.subsystems;

import java.util.List;
import java.util.function.Consumer;

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
    
    protected void apply(Consumer<NAR_Motor> action) {
        motors.forEach(action);
    }

    protected Command applyCommand(Consumer<NAR_Motor> action) {
        return runOnce(()-> apply(action));
    }
    /**
     * Sets power to motor.
     * 
     * @param power Setpoint the pivot goes to.
     */
    public void run(double power) {
        apply(motor -> motor.set(power));
    }
    public Command runCommand(double power) {
        return applyCommand(motor -> motor.set(power));
    }


    public void runVolts(double volts) {
        apply(motor -> motor.setVolts(volts));
    }
    public Command runVoltsCommand(double volts) {
        return applyCommand(motor -> motor.setVolts(volts));
    }

    public double getVolts() {
        return motors.get(0).getAppliedOutput() * 12;
    }

    /**
     * Stops all motors in the subsystem.
     */
    public void stop() {
        run(0);
    }
    public Command stopCommand() {
        return runCommand(0);
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

    /**
     * Resets the position of the motor.
     */
    public void reset() {
        apply(motor -> motor.resetPosition(0));
    }

    /**
     * Resets the position of the motor.
     */
    public Command resetCommand() {
        return applyCommand(motor -> motor.resetPosition(0));
    }
}