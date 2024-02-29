package common.core.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.hardware.motorcontroller.NAR_Motor;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Team 3128's Manipulator Template class.
 * @since 2024 Crescendo
 * @author Maggie Liang
 */
public abstract class ManipulatorTemplate extends SubsystemBase {
    
    protected final NAR_Motor[] motors;
    private final double currentThreshold;
    private final double intakePower;
    private final double outtakePower;
    private final double stallPower;
    private final double lagSeconds;
    
    private BooleanSupplier debug;
    private DoubleSupplier powerSetpoint;

    /**
     * Creates an Elevator object.
     * @param currentThreshold Current when object is intook.
     * @param intakePower Intake power.
     * @param outtakePower Outtake power.
     * @param stallPower Stall Power, run when Manipulator has a game piece.
     * @param lagSeconds Time before current check is run.
     * @param motors Elevator motors.
     */
    public ManipulatorTemplate(double currentThreshold, double intakePower, double outtakePower, double stallPower, double lagSeconds, NAR_Motor... motors){
        this.currentThreshold = currentThreshold;
        this.intakePower = intakePower;
        this.outtakePower = outtakePower;
        this.stallPower = stallPower;
        this.lagSeconds = lagSeconds;
        this.motors = motors;
        configMotors();
    }
    
    /**
     * Configure motor settings.
     */
    protected abstract void configMotors();
    
    /**
     * Returns current of the first motor.
     */
    public double getCurrent(){
        return motors[0].getStallCurrent();
    }
    
    /**
     * Returns whether manipulator has an object.
     */
    public boolean hasObjectPresent(){
        return Math.abs(getCurrent()) > currentThreshold;
    }
    
    /**
     * Sets power to all motors.
     *
     * @param power The power of the motor on [-1, 1]
     */
    protected void setPower(double power){
        final double output = (debug != null && debug.getAsBoolean()) ? powerSetpoint.getAsDouble() : power;
        for (final NAR_Motor motor : motors) {
            motor.set(output);
        }
    }
    
    /**
     * Sets manipulator power.
     * @param power Power the motor is run at.
     * @return Command setting manipulator power.
     */
    public Command runManipulator(double power){
        return runOnce(()-> setPower(power));
    }
    
    /**
     * Intake a game piece
     * @return Command to intake a game piece.
     */
    public Command intake() {
        return sequence(
            runManipulator(intakePower),
            waitSeconds(lagSeconds),
            waitUntil(()-> hasObjectPresent()),
            runOnce(()-> setPower(stallPower))
        );
    }
    
    /**
     * Outtake a game piece.
     * @return Command to outtake a game piece.
     */
    public Command outtake() {
        return runManipulator(outtakePower);
    }
    
    /**
     * Initializes shuffleboard with debug elements.
     */
    public void initShuffleboard() {
        NAR_Shuffleboard.addData(getName(), "Object Present", ()-> hasObjectPresent(), 0, 0);
        NAR_Shuffleboard.addData(getName(), "Manip current", () -> getCurrent(), 0, 1);
        NAR_Shuffleboard.addData(getName(), "TOGGLE", false, 1, 0).withWidget("Toggle Button").getEntry();
        debug = NAR_Shuffleboard.getBoolean(getName(), "TOGGLE");
        NAR_Shuffleboard.addData(getName(), "DEBUG", ()-> debug.getAsBoolean(), 1, 1);
        powerSetpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 1,2);
    }
}
